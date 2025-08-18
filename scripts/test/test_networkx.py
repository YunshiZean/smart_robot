import networkx as nx
import osmnx as ox
from shapely.geometry import LineString
from shapely.geometry import Point

G: nx.MultiDiGraph
osm_filename = (
            r"/home/ros/ros_ws/smart_robot_ws/src/autonomous_driving/map/way2025.osm"
        )
G = ox.graph.graph_from_xml(osm_filename)
# print(G)
# print(G.nodes)
# print(G.edges)

uvk, data = zip(*G.nodes(data=True))


# convert node x/y attributes to Points for geometry column
node_geoms = (Point(d["x"], d["y"]) for d in data)

uu, vv, data = zip(*G.edges(data=True))
# print(u, v, k)
path = []
for u,v,d in zip(uu,vv,data):    

    if "geometry" in d:
        # if geometry attribute exists, add all its coords to list
        xs, ys = d["geometry"].xy
        for x, y in zip(xs, ys):
            path.append((x, y))
    else:
        # otherwise, the edge is a straight line from node to node
        x = G.nodes[u]["x"]
        y = G.nodes[u]["y"]
        path.append((x, y))

        x = G.nodes[v]["x"]
        y = G.nodes[v]["y"]
        path.append((x, y))
print(path)


