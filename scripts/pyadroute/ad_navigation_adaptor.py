# © Copyright 2025 smg@buu.edu.cn
# SPDX-License-Identifier: GPL-3.0-or-later

import threading
import time

from .node_attribute import NodeAttribute

from .protocols import Position
from .path_wraper import PathWrapper

from .state import State
from .Control.move_to_pose import PoseController
from .PathTracking.pursuit_controller import PursuitController
from .ad_route_adaptor import ADRouteAdaptor
from .utils.path import (
    calc_path_yaw_np,
    calc_path_yaw_scipy,
    pathw_concatenation,
    refine_pathw,
    is_pathw_valid,
    shrink_to_endpoint,
)
from .utils.logger import get_logger
from .obstacle_detect.ad_obstacle_detect_adaptor import AdObstacleDetectAdaptor
import numpy as np
from geometry_msgs.msg import Pose
from turn_on_wheeltec_robot.msg import TrafficSignal, TrafficLights

# Parameters
MIN_TARGET_DISTANCE = 0.03

VEHICLE_WIDTH = 0.20 + 0.04
OBSTACLE_TIMEOUT = 2.0  # 2 second

# WB = 0.175  # [m] wheel base of vehicle.
#             # huaner=0.175 wheeltec=0.162 by smg

logger = get_logger("AdNavigationAdapter")


class AdNavigationAdapter:
    class Option:
        Kp_rho: float
        Kp_alpha: float
        Kd_alpha: float
        Kp_beta: float
        obstacle_dist: float
        MAX_LINEAR_SPEED: float
        MAX_ANGULAR_SPEED: float
        LOOK_AHEAD_DISTANCE: float
        Refine_Path_Step: float

        def __init__(self):
            self.Kp_rho = 2.0
            self.Kp_alpha = 1.0
            self.Kd_alpha = 0.0
            self.Kp_beta = 0.08
            self.obstacle_dist = 0.3
            self.MAX_LINEAR_SPEED = 0.4
            self.MAX_ANGULAR_SPEED = np.radians(45.0)
            self.LOOK_AHEAD_DISTANCE = 0.15
            self.Refine_Path_Step = 0.01

    opt: Option
    pathw: PathWrapper

    route_adaptor: ADRouteAdaptor
    path_tracking: PursuitController
    pose_controller: PoseController
    obstacle_detector: AdObstacleDetectAdaptor

    obstacle_detected: bool
    stop_timeout: int
    tick: int

    def __init__(self, osm_filename, opt: Option = None):
        self.opt = self.Option() if opt is None else opt
        # self.lock = threading.Lock()
        self.route_adaptor = ADRouteAdaptor(osm_filename)
        self.path_tracking = PursuitController(self.opt.LOOK_AHEAD_DISTANCE)
        self.pose_controller = PoseController(
            self.opt.Kp_rho, self.opt.Kp_alpha, self.opt.Kd_alpha, self.opt.Kp_beta
        )  # 9, 15, 3
        self.obstacle_detector = AdObstacleDetectAdaptor(VEHICLE_WIDTH / 2)

        self.pathw = None

        self.traffic_signal = None
        self.last_traffic_light_id = None

        self.obstacle_detected = False
        self.stop_timeout = 0
        self.tick = 0

    # make sure process_loop and make_route running in one thread.
    def process_loop(self, current_pose: Pose):
        state = State(current_pose)

        nearest_point_index, point_target_idx, target_distance = (
            self.path_tracking.pursuit_control(state)
        )
        # print(nearest_point_index, point_target_idx, target_distance)

        self.tick += 1
        if self.tick >= 10:
            self.tick = 0
            self.obstacle_detected = self.obstacle_detector.detect(
                state, nearest_point_index, point_target_idx, self.opt.obstacle_dist
            )
            if self.obstacle_detected:
                # stop car
                self.stop_timeout = time.time()

        if time.time() - self.stop_timeout < OBSTACLE_TIMEOUT:
            rho = v = w = 0
        elif self.check_traffic_signal(point_target_idx):
            rho = v = w = 0
        else:
            rho, v, w = self.move_to_pose(state, point_target_idx, target_distance)

        return (
            rho,
            v,
            w,
            nearest_point_index,
            point_target_idx,
        )

    def move_to_pose(self, state: State, point_target_idx, target_distance):
        rho = v = w = 0

        if self.is_path_valid() and state is not None and point_target_idx is not None:
            target_pose = self.pathw.path[point_target_idx]
            x_goal = target_pose[0]
            y_goal = target_pose[1]

            if target_distance > MIN_TARGET_DISTANCE:
                theta = state.yaw
                x_diff = x_goal - state.x
                y_diff = y_goal - state.y
                theta_goal = self.pathw.yaws[point_target_idx]
                # theta_goal = 0
                rho, v, w = self.pose_controller.calc_control_command(
                    x_diff, y_diff, theta, theta_goal
                )
                # print(theta,theta_goal)
                # print(rho, v, w)

                if abs(v) > self.opt.MAX_LINEAR_SPEED:
                    v = np.sign(v) * self.opt.MAX_LINEAR_SPEED

                if abs(w) > self.opt.MAX_ANGULAR_SPEED:
                    w = np.sign(w) * self.opt.MAX_ANGULAR_SPEED
            else:
                # arrived, navigation over
                self.clear_path()
                logger.info("Target has arrived.")

        return rho, v, w

    def make_route(self, ps: Position, pe: Position):
        route = None
        pathw = None
        position_start = position_end = None
        if ps is not None and pe is not None:
            try:
                route, position_start, position_end = (
                    self.route_adaptor.shortest_path_align_edge(
                        ps,
                        pe,
                    )
                )
                logger.info("route1: %s", route)

                pathw = self.route_adaptor.get_pathw(route)
                pathw = refine_pathw(pathw,self.opt.Refine_Path_Step)
                if is_pathw_valid(pathw):
                    pathw = shrink_to_endpoint(pathw, position_start, position_end)
                    pathw.calc_yaw_np()
            except Exception as e:
                route = None
                pathw = None
                position_start = position_end = None
                logger.exception(e)

        return route, pathw, position_start, position_end

    def make_route_muti_point(self, point_list) -> list:
        pathw_m = None
        positions = []
        if point_list is not None and len(point_list) > 1:
            for ps, pe in zip(point_list[:-1], point_list[1:]):
                _, pw, pos_start, pos_end = self.make_route(ps, pe)
                if is_pathw_valid(pw):
                    pathw_m = pathw_concatenation(pathw_m, pw)
                    if len(positions) == 0:
                        positions.append(pos_start)
                    positions.append(pos_end)

        pathw_m.calc_yaw_np()
        self.pathw = pathw_m

        self.path_tracking.set_pathw(pathw_m)
        self.obstacle_detector.set_pathw(pathw_m, VEHICLE_WIDTH / 2)

        return positions

    def is_path_valid(self):
        return is_pathw_valid(self.pathw)

    def clear_path(self):
        self.pathw = None

    def check_traffic_signal(self, point_idx):
        stop_at_red_light = False
        found = False
        if point_idx is not None and self.is_path_valid():
            tl_id = self.pathw.get_tag_value(
                point_idx, NodeAttribute.TAG_TRAFFIC_LIGHTS_ID
            )

            if tl_id is not None:
                found = True
                tl_id = int(tl_id)
                if self.last_traffic_light_id != tl_id:
                    stop_at_red_light = self.check_red_light(tl_id)
                    if not stop_at_red_light:
                        self.last_traffic_light_id = tl_id

                # logger.info("traffic id=%d, stop=%s", tl_id, stop_at_red_light)

        if not found:
            self.last_traffic_light_id = None

        return stop_at_red_light

    def check_red_light(self, light_id: int):
        stop_at_red_light = False
        msg: TrafficSignal = self.traffic_signal
        # logger.info(msg)
        if msg is not None and msg.lights is not None:
            for tl in msg.lights:
                if tl.id == light_id:
                    if tl.light != tl.GREEN:
                        stop_at_red_light = True
                    break

        return stop_at_red_light

    def set_traffic_signal(self, msg: TrafficSignal):
        self.traffic_signal = msg
