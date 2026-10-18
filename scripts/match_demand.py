"""Match lat/lon trips to the nearest node of a FleetPy network.

Run from the FleetPy repo root:
  python scripts/match_demand.py --network my_city --trips raw_trips.csv --demand-name my_demand --out trips_day1.csv

Input CSV columns: rq_time (seconds from sim start), start_lat, start_lon, end_lat, end_lon
Output: data/demand/<demand-name>/matched/<network>/<out> with rq_time,start,end,request_id
Trips whose start and end snap to the same node are dropped.
"""
import argparse, os
import geopandas as gpd
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--network", required=True)
ap.add_argument("--trips", required=True)
ap.add_argument("--demand-name", required=True)
ap.add_argument("--out", required=True)
args = ap.parse_args()

net = os.path.join("data", "networks", args.network, "base")
crs = open(os.path.join(net, "crs.info")).read().strip()
nodes = pd.read_csv(os.path.join(net, "nodes.csv"))
ng = gpd.GeoDataFrame(nodes[["node_index"]], geometry=gpd.points_from_xy(nodes.pos_x, nodes.pos_y), crs=crs)

trips = pd.read_csv(args.trips)
for side in ["start", "end"]:
    pts = gpd.GeoDataFrame(geometry=gpd.points_from_xy(trips[f"{side}_lon"], trips[f"{side}_lat"]),
                           crs="EPSG:4326").to_crs(crs)
    j = gpd.sjoin_nearest(pts, ng, how="left", distance_col="snap_m")
    j = j[~j.index.duplicated()]
    trips[side] = j["node_index"].values
    trips[f"{side}_snap_m"] = j["snap_m"].values

trips = trips[trips.start != trips.end].sort_values("rq_time").reset_index(drop=True)
trips["request_id"] = range(len(trips))
out_dir = os.path.join("data", "demand", args.demand_name, "matched", args.network)
os.makedirs(out_dir, exist_ok=True)
trips[["rq_time", "start", "end", "request_id"]].astype(int).to_csv(os.path.join(out_dir, args.out), index=False)
print(f"{len(trips)} trips written; snap distance (m) median {trips.start_snap_m.median():.0f}, "
      f"max {max(trips.start_snap_m.max(), trips.end_snap_m.max()):.0f}")
