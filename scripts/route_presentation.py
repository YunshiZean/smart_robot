#!/usr/bin/env python3
# encoding: utf-8

# © Copyright 2025 smg@buu.edu.cn
# SPDX-License-Identifier: GPL-3.0-or-later

import datetime
import os
from pyadroute.path_wraper import PathWrapper
import pyadroute.node_attribute as na
from pyadroute.utils.path import path_concatenation, pathw_concatenation, is_pathw_valid
import ros_helper as roser
import rospy
import threading

# import pyroutelib3
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from pyadroute.ad_navigation_adaptor import AdNavigationAdapter
import numpy as np


class RoutePresentationNode:
    navigation: AdNavigationAdapter
    pathw: PathWrapper

    def __init__(self, name):

        rospy.init_node(name)

        self.loop_rate = rospy.get_param("~loop_rate", 100)
        self.txt_dir = rospy.get_param("~txt_dir", "")
        osm_filename = rospy.get_param("~osm_filename", "")

        if len(self.txt_dir) > 0 and not os.path.exists(self.txt_dir):
            os.makedirs(self.txt_dir)

        opt = AdNavigationAdapter.Option()
        opt.Refine_Path_Step = 0.02

        self.navigation = AdNavigationAdapter(osm_filename, opt)
        print(self.navigation.route_adaptor.G)
        print(self.navigation.route_adaptor.G.graph)

        self.osm_msg = roser.create_osm_ros_MarkerArray2(
            self.navigation.route_adaptor.G
        )

        self.click_point = None
        self.click_route_end_point = None
        self.path_point_list = []
        self.point_start = None
        self.pathw = None

        self.pub_osm = rospy.Publisher("/graph_osm", MarkerArray, queue_size=1)

        self.pub_path = rospy.Publisher("/path", Path, queue_size=1)
        self.pub_point = rospy.Publisher("/path_endpoint", MarkerArray, queue_size=1)

        self.sub_route_end = rospy.Subscriber(
            "/route/rviz/route_end", PointStamped, self.callback_route_end
        )
        self.sub_rviz = rospy.Subscriber("/clicked_point", PointStamped, self.callback)

    def run(self):
        pub_osm_thread = threading.Thread(target=self.thread_publish_osm)
        pub_osm_thread.start()

        rospy.loginfo("------Navigation Node start-------")

        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            self.process_loop()

            rate.sleep()

        pub_osm_thread.join()

        rospy.loginfo("------Navigation Node stop-------")

    def process_loop(self):
        point_new = self.click_point
        route_end = self.click_route_end_point

        if point_new is not None:
            self.click_point = None
            if self.point_start is not None:
                self.make_path(self.point_start, point_new)
            else:
                self.public_path_endpoint_delete_all()

            self.point_start = point_new

        if route_end is not None:
            # print(self.path)
            print(self.pathw.tags)
            self.save_to_txt()

            self.click_route_end_point = None
            self.point_start = None
            self.pathw = None
            self.path_point_list = []

            rospy.loginfo("Making route-path was ended.")

    def make_path(self, point_start, point_end):
        if point_start is not None and point_end is not None:
            route, pathw, position_start, position_end = self.navigation.make_route(
                point_start, point_end
            )
            if is_pathw_valid(pathw):
                self.pathw = pathw_concatenation(self.pathw, pathw)
                if self.pathw is not None:
                    if len(self.path_point_list) == 0:
                        self.path_point_list.append(position_start)
                    self.path_point_list.append(position_end)
                    # print(self.pathw.path)
                    # print(self.path_point_list)
                    self.publish_path(self.pathw.path)
                    self.publish_path_endpoint(self.path_point_list)

    def need_back_car(self):
        back_car = False
        mode2_start_idx = None
        if self.pathw.tags[-1].get_tag_value("park", "0") == "1":
            # for tag in self.pathw.tags[::-1]:
            for idx in range(len(self.pathw.tags) - 1, -1, -1):
                tag = self.pathw.tags[idx]
                if tag.get_tag_value(tag.TAG_PARK, "0") == "1":
                    continue
                else:
                    if tag.get_tag_value(tag.TAG_MODE2, "0") == "5":
                        back_car = True
                        mode2_start_idx = idx
                    else:
                        break

        return back_car, mode2_start_idx

    # time,x,y,z,yaw,lat,lon,heading,gpsx,gpsy,gpsyaw,mode1,mode2,mode3
    def save_to_txt(self):
        formatted_datetime = datetime.datetime.now().strftime("%Y%m%d%H%M%S.txt")
        txt_filename = os.path.join(self.txt_dir, formatted_datetime)
        print(txt_filename)
        str_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        back_car, mode2_start_idx = self.need_back_car()
        print("back_car is ", back_car, mode2_start_idx)
        with open(txt_filename, "w") as f:
            for idx in range(len(self.pathw.path)):
                p = self.pathw.path[idx]
                tag = self.pathw.tags[idx]
                # for p, tag in zip(list(range(len())),self.pathw.path, self.pathw.tags):
                f.write("%s,%.3f,%.3f,0,0,0,0,0,0,0,0," % (str_time, p[0], p[1]))
                if back_car:
                    if idx < mode2_start_idx:
                        f.write("%s,0,0\n" % (tag.get_tag_value(tag.TAG_MODE1)))
                    else:
                        f.write(
                            "%s,%s,0\n"
                            % (
                                tag.get_tag_value(tag.TAG_MODE1),
                                tag.get_tag_value(tag.TAG_MODE2),
                            )
                        )
                else:
                    f.write("%s,0,0\n" % (tag.get_tag_value(tag.TAG_MODE1)))

    def callback_route_end(self, msg: PointStamped):
        self.click_route_end_point = (msg.point.x, msg.point.y)

    def callback(self, msg: PointStamped):
        self.click_point = (msg.point.x, msg.point.y)
        rospy.loginfo("clicked point: (%.3f, %.3f)", msg.point.x, msg.point.y)

    def thread_publish_osm(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.osm_msg is not None:
                self.pub_osm.publish(self.osm_msg)

            rate.sleep()

    def publish_path(self, path):
        msg = roser.create_path_msg(path)
        if msg is not None:
            self.pub_path.publish(msg)

    def publish_path_endpoint(self, positions):
        msg = roser.create_path_endpoint_MarkerArray(positions)
        if msg is not None:
            self.pub_point.publish(msg)

    def public_path_endpoint_delete_all(self):
        msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = "path_endpoint"
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self.pub_point.publish(msg)


if __name__ == "__main__":
    node = RoutePresentationNode("RoutePresentationNode")
    node.run()
