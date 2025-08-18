#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import rospy
import threading
import pyroutelib3
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import tf_conversions
from pyroutelib3.protocols import DistanceFunction, Position
from pyroutelib3.distance import euclidean_distance


def my_euclidean_distance(a: Position, b: Position) -> float:
    return math.dist(a, b)


class RouteNode:
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name)

        self.pub = rospy.Publisher("/path", Path, queue_size=1)
        self.pub_point = rospy.Publisher("/path_endpoint", MarkerArray, queue_size=1)

        self.pub_route = rospy.Publisher("/osm_route", MarkerArray, queue_size=1)
        # self.point_pub = rospy.Publisher("point", PointStamped, queue_size=1)
        self.sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.position_start = None
        self.position_end = None

        self.position_start_copy = None
        self.position_end_copy = None

        osm_filename = (
            r"/home/ros/ros_ws/smart_robot_ws/src/autonomous_driving/map/way.osm"
        )
        with open(osm_filename, "r") as f:
            self.graph = pyroutelib3.osm.Graph.from_file(
                pyroutelib3.osm.CarProfile(), f, format="xml"
            )

        # print(self.graph.edges)

        self.init_osm()

        # self.pub_osm()

        # rospy.spin()

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pub_osm()
            self.pub_path_point()
            rate.sleep()

        rospy.loginfo("------Route Node stop-------")

    def callback(self, msg):
        if self.position_start is None:
            self.position_start = (msg.point.x, msg.point.y)
        else:
            self.position_end = (msg.point.x, msg.point.y)

            # start_node = self.graph.find_nearest_node(
            #     self.position_start, distance=euclidean_distance
            # )
            # end_node = self.graph.find_nearest_node(
            #     self.position_end, distance=euclidean_distance
            # )

            # print(start_node)
            # print(end_node)

            start_node, _ = self.graph.find_nearest_edge(self.position_start)
            _, end_node = self.graph.find_nearest_edge(self.position_end)

            print(start_node)
            print(end_node)

            # route = pyroutelib3.find_route(graph, start_node, end_node)
            self.route = pyroutelib3.find_route_without_turn_around(
                self.graph, start_node.id, end_node.id, distance=euclidean_distance
            )
            # print("route forward:", self.route)
            # self.route = pyroutelib3.find_route_without_turn_around(
            #     self.graph, end_node.id, start_node.id
            # )
            # print("route backward:", self.route)

            # route_lat_lons = [self.graph.get_node(node).position for node in self.route]
            # print(route_lat_lons)

            self.position_start_copy = self.position_start
            self.position_end_copy = self.position_end

            self.position_start = None
            self.position_end = None

            self.pub_path()

    def init_osm(self):
        self.osm_msg = MarkerArray()
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = "map"
        id = 0
        for way in self.graph.grapher_builder.way_nodes:
            id += 1
            line_strip = Marker()
            line_strip.id = id
            line_strip.type = Marker.LINE_STRIP
            line_strip.header = header
            line_strip.ns = "osm_route"
            line_strip.action = Marker.ADD
            line_strip.pose.orientation.w = 1.0
            line_strip.scale.x = 0.01
            line_strip.color.r = 0.5
            line_strip.color.g = 0.5
            line_strip.color.b = 0.5
            line_strip.color.a = 1

            for node in self.graph.grapher_builder.way_nodes[way]:
                pt = self.graph.get_node(node).position
                ps = Point()

                ps.x = pt[0]
                ps.y = pt[1]
                ps.z = 0

                line_strip.points.append(ps)

            self.osm_msg.markers.append(line_strip)

    def pub_path_point(self):
        if self.position_start_copy is None or self.position_end_copy is None:
            return

        msg = MarkerArray()
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = "map"
        id = 0
        for ii in range(2):
            id += 1
            point = Marker()
            point.id = id
            point.type = Marker.SPHERE
            point.header = header
            point.ns = "path_point"
            point.action = Marker.ADD
            point.pose.orientation.w = 1.0
            point.scale.x = 0.108
            point.scale.y = 0.108
            point.scale.z = 0.108
            if ii == 0:
                point.color.r = 1.0
                point.color.g = 0.0
                point.color.b = 0.0
                point.color.a = 1

                pt = self.position_start_copy
            else:
                point.color.r = 0.0
                point.color.g = 0.0
                point.color.b = 1.0
                point.color.a = 1

                pt = self.position_end_copy


            point.pose.position.x = pt[0]
            point.pose.position.y = pt[1]
            point.pose.position.z = 0


            # ps = Point()
            # ps.x = pt[0]
            # ps.y = pt[1]
            # ps.z = 0
            # point.points.append(ps)

            msg.markers.append(point)

        self.pub_point.publish(msg)

    def pub_osm(self):
        self.pub_route.publish(self.osm_msg)

    def pub_path(self):
        msg = Path()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"

        for id in range(len(self.route)):
            node = self.route[id]
            # if id == 0:
            #     pt = self.position_start_copy
            # elif id == len(self.route) - 1:
            #     pt = self.position_end_copy
            # else:
            #     pt = self.graph.get_node(node).position

            pt = self.graph.get_node(node).position

            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            ps.pose.position.z = 0

            quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            ps.pose.orientation.x = quat[0]
            ps.pose.orientation.y = quat[1]
            ps.pose.orientation.z = quat[2]
            ps.pose.orientation.w = quat[3]

            msg.poses.append(ps)

        self.pub.publish(msg)


if __name__ == "__main__":
    RouteNode("Route_Node")
