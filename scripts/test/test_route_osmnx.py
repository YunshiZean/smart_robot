#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import rospy
import threading
import osmnx as ox
from osmnx import projection
import networkx as nx
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
from pyadroute import ADRouteAdaptor
from pyadroute.utils import convert_position_xy, refine_path, shrink_path_to_endpoint


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

        self.route = None
        self.path = None

        osm_filename = (
            r"/home/ros/ros_ws/smart_robot_ws/src/autonomous_driving/map/way.osm"
        )

        with open(osm_filename, "r") as f:
            self.graph = pyroutelib3.osm.Graph.from_file(
                pyroutelib3.osm.CarProfile(), f, format="xml"
            )

        self.route_adaptor = ADRouteAdaptor(osm_filename)

        print(self.route_adaptor.G.graph)

        self.init_osm_ros()

        rospy.loginfo("------Route Node start-------")

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pub_osm()
            rate.sleep()

        rospy.loginfo("------Route Node stop-------")

    def callback(self, msg):
        if self.position_start is None:
            self.position_start = (msg.point.x, msg.point.y)
        else:
            self.position_end = (msg.point.x, msg.point.y)

            self.route, self.position_start_copy, self.position_end_copy = (
                self.route_adaptor.shortest_path_align_edge(
                    self.position_start,
                    self.position_end,
                )
            )

            self.path = self.route_adaptor.get_path(self.route)
            self.path = refine_path(self.path)
            self.path = shrink_path_to_endpoint(
                self.position_start_copy, self.position_end_copy, self.path
            )

            print("route:", self.route)

            # self.position_start_copy = self.position_start
            # self.position_end_copy = self.position_end
            self.position_start = None
            self.position_end = None

            self.pub_path()
            self.pub_path_endpoint()

    def init_osm_ros(self):
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

    def pub_osm(self):
        self.pub_route.publish(self.osm_msg)

    def create_pose_msg2(self, x, y, header):
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

    def pub_path(self):

        if self.path is None:
            return

        msg = Path()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"

        for pos in self.path:
            ps = self.create_pose_msg(pos[0], pos[1], msg.header)
            msg.poses.append(ps)

        self.pub.publish(msg)

    def pub_path_endpoint(self):
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
            point.ns = "path_endpoint"
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

            msg.markers.append(point)

        self.pub_point.publish(msg)


if __name__ == "__main__":
    RouteNode("Route_Node")
