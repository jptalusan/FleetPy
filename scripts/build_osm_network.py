"""Build a FleetPy road network from OpenStreetMap.

Run from the FleetPy repo root, for example:
  python scripts/build_osm_network.py --name my_city --place "Some City, State, USA" --crs EPSG:32616 --units mph
  python scripts/build_osm_network.py --name my_area --bbox -84.40 33.74 -84.36 33.78 --crs EPSG:32616 --units mph

Notes:
- Needs osmnx 2.x and internet access to OpenStreetMap.
- Loads the graph with osmnx directly and passes it to FleetPy's converter (osmnG=...),
  because the converter's own download call uses the old osmnx 1.x bbox signature,
  and its from_crs_to_crs option calls a method that does not exist.
- Projects to a metric CRS (pick the UTM zone for your area) and writes crs.info,
  which the converter does not write.
"""
import argparse, contextlib, io, os, sys
import osmnx as ox

ROOT = os.getcwd()
sys.path.append(ROOT)
sys.path.append(os.path.join(ROOT, "src", "preprocessing", "networks", "extractors"))
from osm_converter import createNetwork  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--name", required=True, help="network_name used in FleetPy configs")
area = ap.add_mutually_exclusive_group(required=True)
area.add_argument("--place", help='place name osmnx understands, e.g. "Downtown, City, State, USA"')
area.add_argument("--bbox", nargs=4, type=float, metavar=("WEST", "SOUTH", "EAST", "NORTH"))
ap.add_argument("--crs", required=True, help="metric CRS, e.g. EPSG:32616 (UTM 16N)")
ap.add_argument("--units", default="mph", choices=["mph", "kmh"], help="unit of OSM maxspeed tags")
ap.add_argument("--network-type", default="drive")
args = ap.parse_args()

if args.place:
    G = ox.graph_from_place(args.place, network_type=args.network_type)
else:
    G = ox.graph_from_bbox(tuple(args.bbox), network_type=args.network_type)

G = ox.truncate.largest_component(G, strongly=True)  # every node reachable from every other
G = ox.project_graph(G, to_crs=args.crs)
print(f"nodes: {len(G.nodes)}, edges: {len(G.edges)}")

with contextlib.redirect_stdout(io.StringIO()):  # converter prints every edge
    createNetwork(args.name, osmnG=G, unit_speeds=args.units)

base = os.path.join(ROOT, "data", "networks", args.name, "base")
with open(os.path.join(base, "crs.info"), "w") as f:
    f.write(args.crs)
print(f"written to {base}")
