#!/usr/bin/env python3
# encoding: utf-8
# © Copyright 2025 smg@buu.edu.cn
# SPDX-License-Identifier: GPL-3.0-or-later

# import pyroutelib3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import tf_conversions
import networkx as nx


# def create_osm_ros_MarkerArray(graph: pyroutelib3.osm.Graph):
#     osm_msg = MarkerArray()
#     header = Header()
#     header.stamp = rospy.get_rostime()
#     header.frame_id = "map"
#     id = 0
#     for way in graph.grapher_builder.way_nodes:
#         id += 1
#         line_strip = Marker()
#         line_strip.id = id
#         line_strip.type = Marker.LINE_STRIP
#         line_strip.header = header
#         line_strip.ns = "osm_route"
#         line_strip.action = Marker.ADD
#         line_strip.pose.orientation.w = 1.0
#         line_strip.scale.x = 0.01
#         line_strip.color.r = 0.5
#         line_strip.color.g = 0.5
#         line_strip.color.b = 0.5
#         line_strip.color.a = 1

#         for node in graph.grapher_builder.way_nodes[way]:
#             pt = graph.get_node(node).position
#             ps = Point()

#             ps.x = pt[0]
#             ps.y = pt[1]
#             ps.z = 0

#             line_strip.points.append(ps)

#         osm_msg.markers.append(line_strip)
#     return osm_msg


def create_osm_ros_MarkerArray2(G: nx.MultiDiGraph):
    if G is None:
        return None

    osm_msg = MarkerArray()
    header = Header()
    header.stamp = rospy.get_rostime()
    header.frame_id = "map"
    id = 0
    # uu, vv, data = zip(*G.edges(data=True))
    for u, v, d in G.edges(data=True):
        id += 1
        line_strip = Marker()
        line_strip.id = id
        line_strip.type = Marker.LINE_STRIP
        line_strip.header = header
        line_strip.ns = "graph_osm"
        line_strip.action = Marker.ADD
        line_strip.pose.orientation.w = 1.0
        line_strip.scale.x = 0.01
        line_strip.color.r = 0.5
        line_strip.color.g = 0.5
        line_strip.color.b = 0.5
        line_strip.color.a = 1

        if "geometry" in d:
            # if geometry attribute exists, add all its coords to list
            xs, ys = d["geometry"].xy
            for x, y in zip(xs, ys):
                ps = Point(x, y, 0)
                line_strip.points.append(ps)
        else:
            # otherwise, the edge is a straight line from node to node
            x = G.nodes[u]["x"]
            y = G.nodes[u]["y"]
            ps = Point(x, y, 0)
            line_strip.points.append(ps)

            x = G.nodes[v]["x"]
            y = G.nodes[v]["y"]
            ps = Point(x, y, 0)
            line_strip.points.append(ps)

        osm_msg.markers.append(line_strip)
    return osm_msg


def create_pose_msg(x, y, header: Header):
    ps = PoseStamped()
    ps.header = header
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0

    quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    ps.pose.orientation.x = quat[0]
    ps.pose.orientation.y = quat[1]
    ps.pose.orientation.z = quat[2]
    ps.pose.orientation.w = quat[3]

    return ps


def create_path_endpoint_MarkerArray(points):
    if points is None:
        return None

    msg = MarkerArray()
    header = Header()
    header.stamp = rospy.get_rostime()
    header.frame_id = "map"

    for ii in range(len(points)):
        point = Marker()
        point.id = ii + 1
        point.type = Marker.SPHERE
        point.header = header
        point.ns = "path_endpoint"
        point.action = Marker.ADD
        point.pose.orientation.w = 1.0
        point.scale.x = 0.108
        point.scale.y = 0.108
        point.scale.z = 0.108

        pt = points[ii]

        if ii % 2 == 0:
            point.color.r = 1.0
            point.color.g = 0.0
            point.color.b = 0.0
            point.color.a = 1
        else:
            point.color.r = 0.0
            point.color.g = 0.0
            point.color.b = 1.0
            point.color.a = 1

        point.pose.position.x = pt[0]
        point.pose.position.y = pt[1]
        point.pose.position.z = 0

        msg.markers.append(point)
    return msg


def create_path_msg(path):

    if path is None:
        return None

    msg = Path()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"

    for pos in path:
        # print(pos)
        ps = create_pose_msg(pos[0], pos[1], msg.header)
        msg.poses.append(ps)

    return msg


def create_line_strip_msg(lines):

    if lines is None:
        return None

    msg = Marker()
    msg.id = 1
    msg.type = Marker.LINE_STRIP
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.ns = "obstacle_area"
    msg.action = Marker.ADD
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.01
    msg.color.r = 1.0
    msg.color.g = 0.0
    msg.color.b = 0.0
    msg.color.a = 1

    for p in lines:
        ps = Point(p[0], p[1], 0)
        msg.points.append(ps)

    return msg


def create_path_msg2(cx, cy):
    if cx is None:
        return None

    msg = Path()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"

    for x, y in zip(cx, cy):
        ps = create_pose_msg(x, y, msg.header)
        msg.poses.append(ps)

    return msg
