"""Build a FleetPy road network from an OSRM server instead of an OpenStreetMap download.

Run from the repo root, for example:
  uv run python scripts/build_osrm_network.py --name some_area --bbox -86.835 36.130 -86.755 36.185 --crs EPSG:32616
  uv run python scripts/build_osrm_network.py --name carta_go --zone zone.geojson --buffer 300 \
      --keep-point -85.269 35.057 --crs EPSG:32616 --spacing 300

How it works:
1. Lay a square lattice of points over the bounding box, or over a polygon from a GeoJSON file (`--zone`, grown by
   `--buffer` metres), `--spacing` metres apart. Points given with `--keep-point` (a depot, for example) are always
   added, even outside the area.
2. Snap each point to the nearest non-motorway road with OSRM. Points further than `--max-snap` from a road
   (river, parks, rail yards) are dropped, and points that snap to the same spot are merged.
3. Ask OSRM for the full travel-time and distance matrix between the snapped points.
4. Drop nodes whose direct OSRM times disagree badly with routes through other nodes (see `drop_detour_outliers`),
   then repair the matrix so the triangle inequality holds exactly (see `shortest_path_closure`).
5. Write it in FleetPy's format for the `NetworkTTMatrix` routing module: every node pair is a direct edge whose
   travel time is the OSRM fastest-route time. Vehicles therefore drive OSRM times; their drawn positions between
   nodes are straight lines.

Outputs in data/networks/<name>/:
  base/nodes.csv, base/edges.csv, base/crs.info, ff/tables/nn_fastest_tt.npy, ff/tables/nn_fastest_distance.npy
  base/keep_points.csv   node index of every --keep-point (lon, lat, node_index)
"""
import argparse
import os
import shutil
import tempfile
from concurrent.futures import ThreadPoolExecutor

import geopandas as gpd
import numpy as np
import pandas as pd
from pyproj import Transformer
from shapely.geometry import Point

import osrm_client
from common import ROOT, setup_logging


def lattice(bbox, crs, spacing, area=None):
    """Return (lon, lat) points of a square lattice with `spacing` metres between points, covering bbox.

    :param area: optional shapely polygon in `crs`; only lattice points inside it are kept
    """
    west, south, east, north = bbox
    to_xy = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    to_lonlat = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    xs, ys = to_xy.transform([west, east, west, east], [south, south, north, north])
    gx, gy = np.meshgrid(np.arange(min(xs), max(xs) + 1, spacing), np.arange(min(ys), max(ys) + 1, spacing))
    gx, gy = gx.ravel(), gy.ravel()
    if area is not None:
        inside = np.array([area.contains(Point(x, y)) for x, y in zip(gx, gy)])
        gx, gy = gx[inside], gy[inside]
    lons, lats = to_lonlat.transform(gx, gy)
    return list(zip(lons, lats))


def zone_area(path, feature, buffer_m, crs):
    """The polygon of one GeoJSON feature in `crs`, grown by buffer_m metres, and its bounding box in lon/lat."""
    zones = gpd.read_file(path)
    if not 0 <= feature < len(zones):
        raise ValueError(f"{path} has {len(zones)} features, --zone-feature {feature} does not exist")
    shape = zones.iloc[[feature]].to_crs(crs).geometry.iloc[0].buffer(buffer_m)
    bbox = gpd.GeoSeries([shape], crs=crs).to_crs("EPSG:4326").total_bounds
    return shape, tuple(bbox)


def snap_points(points, max_snap, log, n_keep=0):
    """Snap points to roads (motorways excluded so nobody is picked up on an interstate).

    The first n_keep points are --keep-points: never dropped for distance, and kept first so merging keeps them.

    :return: DataFrame with lon, lat, snap_m, hint; one row per distinct snapped location
    """
    with ThreadPoolExecutor(8) as pool:
        rows = list(pool.map(lambda p: osrm_client.nearest(p[0], p[1], exclude="motorway"), points))
    snapped = pd.DataFrame(rows, columns=["lon", "lat", "snap_m", "hint"])
    far = (snapped.snap_m > max_snap) & (snapped.index >= n_keep)
    for i in snapped.index[:n_keep]:
        if snapped.snap_m[i] > max_snap:
            log.warning(f"keep-point {points[i]} is {snapped.snap_m[i]:.0f} m from the nearest road; kept anyway")
    snapped = snapped[~far]
    key = snapped.lon.round(5).astype(str) + "," + snapped.lat.round(5).astype(str)  # ~1 m
    dup = key.duplicated()
    if dup[snapped.index < n_keep].any():
        raise RuntimeError("two --keep-points snapped to the same road spot; give points further apart")
    snapped = snapped[~dup].reset_index(drop=True)
    log.info(f"lattice {len(points)} points: {int(far.sum())} dropped (more than {max_snap:.0f} m from a road), "
             f"{int(dup.sum())} merged (same snapped spot), {len(snapped)} nodes kept")
    return snapped


def drop_unroutable(tt, dist, log):
    """Remove nodes until every pair is routable, removing the node with the most unroutable pairs first.

    :return: (tt, dist, indices of kept nodes)
    """
    keep = np.arange(len(tt))
    while True:
        bad = np.isnan(tt[np.ix_(keep, keep)])
        if not bad.any():
            break
        worst = np.argmax(bad.sum(axis=0) + bad.sum(axis=1))
        log.warning(f"node {keep[worst]} has {int(bad[worst].sum() + bad[:, worst].sum())} unroutable pairs, dropped")
        keep = np.delete(keep, worst)
    return tt[np.ix_(keep, keep)], dist[np.ix_(keep, keep)], keep


def shortest_path_closure(tt, dist):
    """Make tt satisfy tt[i,j] <= tt[i,k] + tt[k,j] for every k (Floyd-Warshall over the full matrix).

    OSRM snaps each request to a point part-way along a road, and rounds durations to 0.1 s, so going "via" a third
    node can come out slightly faster than the direct answer. FleetPy's NetworkTTMatrix rebuilds routes by walking
    edges whose times add up to the direct time, which needs the inequality to hold. Distances follow the faster path.

    :return: (repaired tt, repaired dist, number of pairs changed, largest time saved in s)
    """
    tt, dist = tt.copy(), dist.copy()
    original = tt.copy()
    for k in range(len(tt)):
        via = tt[:, k, None] + tt[None, k, :]
        better = via < tt
        tt[better] = via[better]
        dist[better] = (dist[:, k, None] + dist[None, k, :])[better]
    changed = original - tt
    return tt, dist, int((changed > 1e-9).sum()), float(changed.max())


def drop_detour_outliers(tt, dist, log, n_protect, threshold_s, max_share):
    """Remove nodes that OSRM can only leave or reach by a long loop.

    For such a node (a point on a one-way ramp or a parking loop, for example) the direct OSRM time is far longer than
    a route through a third node, because that route may turn around at the third node. Repeatedly drop the node with
    the most pairs improved by more than threshold_s, while that node has them on more than max_share of its pairs.
    The first n_protect nodes (keep-points) are never dropped.

    :return: (tt, dist, indices of kept nodes)
    """
    keep = np.arange(len(tt))
    while True:
        sub_tt, sub_d = tt[np.ix_(keep, keep)], dist[np.ix_(keep, keep)]
        fixed, _, _, _ = shortest_path_closure(sub_tt, sub_d)
        big = (sub_tt - fixed) > threshold_s
        count = big.sum(axis=0) + big.sum(axis=1)
        for i in range(n_protect):
            if count[i] / (2 * (len(keep) - 1)) > max_share:
                log.warning(f"keep-point node {i} has {int(count[i])} pairs improved by more than {threshold_s:.0f} s; "
                            f"kept anyway")
        count[:n_protect] = -1
        worst = int(np.argmax(count))
        if count[worst] / (2 * (len(keep) - 1)) <= max_share:
            return sub_tt, sub_d, keep
        log.warning(f"node {keep[worst]} dropped: {int(count[worst])} of its pairs are more than {threshold_s:.0f} s "
                    f"faster through another node")
        keep = np.delete(keep, worst)


def write_network(name, nodes_xy, tt, dist, crs, log, keep_points=None):
    """Write FleetPy network files into a temporary folder, then swap it into data/networks/<name>."""
    final_dir = os.path.join(ROOT, "data", "networks", name)
    os.makedirs(os.path.dirname(final_dir), exist_ok=True)
    tmp_dir = tempfile.mkdtemp(prefix=f".{name}_", dir=os.path.dirname(final_dir))
    base, tables = os.path.join(tmp_dir, "base"), os.path.join(tmp_dir, "ff", "tables")
    os.makedirs(base)
    os.makedirs(tables)

    n = len(tt)
    pd.DataFrame({"node_index": range(n), "is_stop_only": False,
                  "pos_x": nodes_xy[:, 0], "pos_y": nodes_xy[:, 1]}).to_csv(os.path.join(base, "nodes.csv"), index=False)
    frm, to = np.where(~np.eye(n, dtype=bool))  # every ordered pair of distinct nodes
    pd.DataFrame({"from_node": frm, "to_node": to, "distance": dist[frm, to].round(1),
                  "travel_time": tt[frm, to].round(1), "source_edge_id": range(len(frm))}).to_csv(
        os.path.join(base, "edges.csv"), index=False)
    with open(os.path.join(base, "crs.info"), "w") as f:
        f.write(crs)
    if keep_points is not None:
        keep_points.to_csv(os.path.join(base, "keep_points.csv"), index=False)
    np.save(os.path.join(tables, "nn_fastest_tt.npy"), tt)
    np.save(os.path.join(tables, "nn_fastest_distance.npy"), dist)

    # read back what FleetPy will read and compare
    nodes_back = pd.read_csv(os.path.join(base, "nodes.csv"))
    if len(nodes_back) != n or not np.array_equal(np.load(os.path.join(tables, "nn_fastest_tt.npy")), tt):
        raise RuntimeError(f"read-back of {tmp_dir} does not match what was written")

    if os.path.exists(final_dir):
        shutil.rmtree(final_dir)
    os.rename(tmp_dir, final_dir)
    log.info(f"network written to {final_dir}: {n} nodes, {len(frm)} edges")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="network_name used in FleetPy configs")
    area_group = ap.add_mutually_exclusive_group(required=True)
    area_group.add_argument("--bbox", nargs=4, type=float, metavar=("WEST", "SOUTH", "EAST", "NORTH"))
    area_group.add_argument("--zone", help="GeoJSON file; the lattice covers one of its features")
    ap.add_argument("--zone-feature", type=int, default=0, help="index of the feature in --zone (default 0)")
    ap.add_argument("--buffer", type=float, default=0.0, help="grow the --zone polygon by this many metres")
    ap.add_argument("--keep-point", nargs=2, type=float, action="append", default=[], metavar=("LON", "LAT"),
                    help="a point that is always a node (repeatable), e.g. a depot")
    ap.add_argument("--crs", required=True, help="metric CRS, e.g. EPSG:32616 (UTM 16N)")
    ap.add_argument("--spacing", type=float, default=250.0, help="lattice spacing in metres")
    ap.add_argument("--outlier-threshold", type=float, default=120.0,
                    help="seconds a route through another node must save to count against a node (default 120)")
    ap.add_argument("--outlier-share", type=float, default=0.01,
                    help="drop a node when more than this share of its pairs count against it (default 0.01)")
    ap.add_argument("--max-snap", type=float, help="drop lattice points further than this from a road "
                                                   "(metres, default spacing / 2)")
    args = ap.parse_args()
    if args.bbox is not None:
        west, south, east, north = args.bbox
        if not (west < east and south < north):
            ap.error(f"--bbox must be WEST SOUTH EAST NORTH with WEST < EAST and SOUTH < NORTH, got {args.bbox}")
    if args.buffer < 0:
        ap.error(f"--buffer must be >= 0, got {args.buffer}")
    if args.spacing <= 0:
        ap.error(f"--spacing must be positive, got {args.spacing}")
    max_snap = args.max_snap if args.max_snap is not None else args.spacing / 2

    log = setup_logging("build_osrm_network")
    log.info(f"settings: {vars(args)}, OSRM at {osrm_client.osrm_url()}")

    if args.zone:
        area, bbox = zone_area(args.zone, args.zone_feature, args.buffer, args.crs)
        log.info(f"zone {args.zone} feature {args.zone_feature}, buffer {args.buffer:.0f} m, area {area.area / 1e6:.1f} km2")
    else:
        area, bbox = None, tuple(args.bbox)
    keep_points = [tuple(p) for p in args.keep_point]
    nodes = snap_points(keep_points + lattice(bbox, args.crs, args.spacing, area), max_snap, log, n_keep=len(keep_points))
    tt, dist = osrm_client.table(list(zip(nodes.lon, nodes.lat)), hints=list(nodes.hint))
    tt, dist, keep = drop_unroutable(tt, dist, log)
    lost = [keep_points[i] for i in range(len(keep_points)) if i not in set(keep)]
    if lost:
        raise RuntimeError(f"keep-points {lost} cannot reach the rest of the network and were dropped")
    nodes = nodes.iloc[keep].reset_index(drop=True)  # keep-points stay at indices 0 .. len(keep_points)-1
    tt, dist, keep = drop_detour_outliers(tt, dist, log, len(keep_points), args.outlier_threshold, args.outlier_share)
    nodes = nodes.iloc[keep].reset_index(drop=True)
    tt, dist, n_changed, max_saved = shortest_path_closure(tt, dist)
    log.info(f"triangle repair changed {n_changed} of {tt.size} pairs, largest change {max_saved:.1f} s")
    log.info(f"travel time between nodes: median {np.median(tt):.0f} s, max {tt.max():.0f} s; "
             f"snap distance median {nodes.snap_m.median():.0f} m")

    to_xy = Transformer.from_crs("EPSG:4326", args.crs, always_xy=True)
    xy = np.column_stack(to_xy.transform(nodes.lon.values, nodes.lat.values))
    kept = pd.DataFrame({"lon": [p[0] for p in keep_points], "lat": [p[1] for p in keep_points],
                         "node_index": range(len(keep_points))})
    write_network(args.name, xy, tt, dist, args.crs, log, keep_points=kept)


if __name__ == "__main__":
    main()
