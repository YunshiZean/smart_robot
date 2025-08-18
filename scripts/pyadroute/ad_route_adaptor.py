# © Copyright 2025 smg@buu.edu.cn
# SPDX-License-Identifier: GPL-3.0-or-later

import networkx as nx

from .path_wraper import PathWrapper

from .node_attribute import NodeAttribute
from .utils.path import nearest_point_of_point_to_path, refine_path, distance_ahead_path
import osmnx as ox
from osmnx import projection
from .protocols import DistanceFunction, Position
import numpy as np


class ADRouteAdaptor:
    G: nx.MultiDiGraph

    def __init__(self, osm_filename):
        print("osmnx", ox.__version__)

        ox.settings.useful_tags_way += NodeAttribute.TAG_NAMES

        self.G = ox.graph.graph_from_xml(osm_filename, simplify=False)
        # self.G = projection.project_graph(self.G, to_crs=self.G.graph["crs"])
        # ox.distance.add_edge_lengths(self.G)

        # attr = self.edge_attributes(-50126, -50125, "mode1")
        # print(attr)

    def edge_attributes(self, node1, node2, tag_name="oneway"):
        # attr = nx.get_edge_attributes(self.G, "oneway")
        # print(attr)
        attr = self.G.get_edge_data(node1, node2, 0)
        if tag_name in attr:
            return attr[tag_name]
        else:
            return None

    def edge_all_tags(self, node1, node2):
        na = NodeAttribute()
        for key in NodeAttribute.TAG_NAMES:
            value = self.edge_attributes(node1, node2, key)
            if value is not None:
                na.add_tag(key, value)

        return na

    def nearest_node(self, position, return_dist=False):
        return ox.distance.nearest_nodes(
            self.G, position[0], position[1], return_dist=return_dist
        )

    def shortest_path(self, orin, dest):
        origin_node = ox.distance.nearest_nodes(self.G, orin[0], orin[1])
        destination_node = ox.distance.nearest_nodes(self.G, dest[0], dest[1])

        route = ox.routing.shortest_path(self.G, origin_node, destination_node)
        # self.route = nx.shortest_path(self.G, origin_node, destination_node, weight="length")
        # self.route = nx.astar_path(self.G, origin_node, destination_node, weight="length")
        # self.route = nx.dijkstra_path(
        #     self.G, origin_node, destination_node, weight="length"
        # )

        return route

    def shortest_path_align_edge(
        self, orin: Position, dest: Position, endpoint_in_edge=True
    ):
        # ne: (u,v,k)
        ne = ox.distance.nearest_edges(self.G, orin[0], orin[1])
        orig_edge = (ne[0], ne[1])

        ne = ox.distance.nearest_edges(self.G, dest[0], dest[1])
        dest_edge = (ne[0], ne[1])

        new_orin_position, new_orin_idx, orin_refined_path = (
            self.get_nearest_positon_on_edge(orin, orig_edge)
        )

        if self.same_edge(orig_edge, dest_edge):
            new_dest_position, new_dest_idx, dest_refined_path = (
                self.get_nearest_positon_on_edge(dest, orig_edge)
            )

            if self.oneway(orig_edge):
                orin_dist_ahead = distance_ahead_path(orin_refined_path, new_orin_idx)
                dest_dist_ahead = distance_ahead_path(dest_refined_path, new_dest_idx)
                if orin_dist_ahead > dest_dist_ahead:
                    route = [orig_edge[0], orig_edge[1]]
                elif orin_dist_ahead < dest_dist_ahead:
                    route = ox.routing.shortest_path(
                        self.G, orig_edge[1], orig_edge[0], weight="length"
                    )
                    route.insert(0, orig_edge[0])
                    route.append(orig_edge[1])
                else:
                    route = None
            else:
                orin_dist_ahead = distance_ahead_path(orin_refined_path, new_orin_idx)
                dest_dist_ahead = distance_ahead_path(dest_refined_path, new_dest_idx)

                if orin_dist_ahead > dest_dist_ahead:
                    route = [orig_edge[0], orig_edge[1]]
                else:
                    route = [orig_edge[1], orig_edge[0]]
        else:
            # route = nx.astar_path(self.G, origin_end_node, destination_start_node, weight="length")
            # route = nx.shortest_path(self.G, origin_node, destination_node, weight="length")
            # route = nx.dijkstra_path(self.G, origin_node, destination_node, weight="length")

            new_dest_position, new_dest_idx, dest_refined_path = (
                self.get_nearest_positon_on_edge(dest, dest_edge)
            )

            if orig_edge[1] == dest_edge[0]:
                route = [orig_edge[0], orig_edge[1], dest_edge[1]]
            else:
                route = ox.routing.shortest_path(
                    self.G, orig_edge[1], dest_edge[0], weight="length"
                )

                if route is not None and len(route) < 2:
                    print("orig_edge:", orig_edge, dest_edge)
                    print("route0:", route)

                # route_length = self.get_route_length(route)  # length on earth.
                # print("Route length: ", route_length)

                if route is not None and len(route) > 0:
                    if not self.same_edge((route[0], route[1]), orig_edge):
                        route.insert(0, orig_edge[0])
                    if not self.same_edge((route[-2], route[-1]), dest_edge):
                        route.append(dest_edge[1])

        if endpoint_in_edge:
            return route, new_orin_position, new_dest_position
        else:
            return route

    def oneway(self, edge):
        oneway = self.edge_attributes(edge[0], edge[1])
        if oneway is not None:
            return oneway
        return False

    def same_edge(self, edge1, edge2):
        if edge1 == edge2:
            return True
        if edge1[0] == edge2[1] and edge1[1] == edge2[0]:
            oneway = self.edge_attributes(edge1[0], edge1[1])
            if oneway is None or not oneway:
                return True
        return False

    # length on earth
    def get_route_length(self, route):
        edge_lengths = ox.routing.route_to_gdf(self.G, route)["length"]
        return sum(edge_lengths)

    def get_path(self, route):
        pathw: PathWrapper = self.get_pathw(route)
        if pathw is None:
            return None, None

        return pathw.path, pathw.tags

    def get_pathw(self, route) -> PathWrapper:
        if route is None or len(route) == 0:
            return None

        pathw = PathWrapper()

        x = self.G.nodes[route[0]]["x"]
        y = self.G.nodes[route[0]]["y"]
        ea = self.edge_all_tags(route[0], route[1])
        pathw.append_point((x, y), ea)

        for u, v in zip(route[:-1], route[1:]):
            # if there are parallel edges, select the shortest in length

            data = min(
                self.G.get_edge_data(u, v).values(),
                key=lambda d: d["length"],
            )
            ea = self.edge_all_tags(u, v)
            if "geometry" in data:
                # if geometry attribute exists, add all its coords to list
                xs, ys = data["geometry"].xy
                for x, y in zip(xs, ys):
                    if (x, y) != pathw.path[-1]:
                        pathw.append_point((x, y), ea)
            else:
                # otherwise, the edge is a straight line from node to node
                x = self.G.nodes[u]["x"]
                y = self.G.nodes[u]["y"]
                if (x, y) != pathw.path[-1]:
                    pathw.append_point((x, y), ea)

                x = self.G.nodes[v]["x"]
                y = self.G.nodes[v]["y"]
                if (x, y) != pathw.path[-1]:
                    pathw.append_point((x, y), ea)

        x = self.G.nodes[route[-1]]["x"]
        y = self.G.nodes[route[-1]]["y"]
        if (x, y) != pathw.path[-1]:
            ea = self.edge_all_tags(route[-2], route[-1])
            pathw.append_point((x, y), ea)

        return pathw

    def get_nearest_positon_on_edge(self, pos, edge):
        path, path_attr = self.get_path(edge)
        refined_path, _ = refine_path(path, path_attr)
        new_position, new_pos_idx = nearest_point_of_point_to_path(pos, refined_path)
        return new_position, new_pos_idx, refined_path

    def get_node_position(self, node_id):
        return self.G.nodes[node_id]["x"], self.G.nodes[node_id]["y"]

    def get_node(self, node_id):
        return self.G.nodes[node_id]
