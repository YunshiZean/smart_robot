# © Copyright 2025 smg@buu.edu.cn
# SPDX-License-Identifier: GPL-3.0-or-later
#0.0.1
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
MIN_TARGET_DISTANCE = 0.05

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
            self.MAX_ANGULAR_SPEED = 0.5#np.radians(45.0)
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

        self.Is_arrive = False

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
        """
        处理车辆控制循环的主要函数，计算车辆的运动控制指令。
        
        该函数在一个线程中运行，负责路径跟踪、障碍物检测和交通信号检查，
        并根据这些信息计算车辆的速度和转向指令。
        
        Args:
            current_pose (Pose): 当前车辆的姿态信息，包含位置和方向
            
        Returns:
            tuple: 包含以下元素的元组：
                - rho (float): 车辆与目标点的距离
                - v (float): 线速度指令
                - w (float): 角速度指令
                - nearest_point_index (int): 路径上最近点的索引
                - point_target_idx (int): 目标点的索引
        """
        state = State(current_pose)

        # 执行路径跟踪控制，获取最近点、目标点和目标距离
        nearest_point_index, point_target_idx, target_distance = (
            self.path_tracking.pursuit_control(state)
        )
        # print(nearest_point_index, point_target_idx, target_distance)

        # 每10个循环周期检测一次障碍物
        self.tick += 1
        if self.tick >= 10:
            self.tick = 0
            self.obstacle_detected = self.obstacle_detector.detect(
                state, nearest_point_index, point_target_idx, self.opt.obstacle_dist
            )
            if self.obstacle_detected:
                # 检测到障碍物时停止车辆
                self.stop_timeout = time.time()

        # 根据不同条件设置车辆运动指令
        if time.time() - self.stop_timeout < OBSTACLE_TIMEOUT:
            # 障碍物超时时间内保持静止
            rho = v = w = 0
        elif self.check_traffic_signal(point_target_idx):
            # 检测到交通信号时停止车辆
            rho = v = w = 0
        else:
            # 正常情况下计算移动到目标点的指令
            rho, v, w = self.move_to_pose(state, point_target_idx, target_distance)

        return (
            rho,
            v,
            w,
            nearest_point_index,
            point_target_idx,
        )

    def move_to_pose(self, state: State, point_target_idx, target_distance):
        """
        控制机器人移动到指定目标位置
        
        参数:
        state: State类型，包含机器人当前状态信息（位置、方向等）
        point_target_idx: 目标点在路径中的索引
        target_distance: 当前位置到目标点的距离
        
        返回值:
        tuple: (rho, v, w) 
                rho: 到目标点的距离
                v: 线速度指令
                w: 角速度指令
        """
        rho = v = w = 0

        # 检查路径有效性以及输入参数是否完整
        if self.is_path_valid() and state is not None and point_target_idx is not None:
            target_pose = self.pathw.path[point_target_idx]
            x_goal = target_pose[0]
            y_goal = target_pose[1]

            # 判断是否到达目标点
            if target_distance > MIN_TARGET_DISTANCE:
                # 未到达目标点，继续导航
                self.Is_arrive = False
                theta = state.yaw
                x_diff = x_goal - state.x
                y_diff = y_goal - state.y
                theta_goal = self.pathw.yaws[point_target_idx]
                # 计算控制指令
                rho, v, w = self.pose_controller.calc_control_command(
                    x_diff, y_diff, theta, theta_goal
                )

                # 限制线速度不超过最大值
                if abs(v) > self.opt.MAX_LINEAR_SPEED:
                    v = np.sign(v) * self.opt.MAX_LINEAR_SPEED

                # 限制角速度不超过最大值
                if abs(w) > self.opt.MAX_ANGULAR_SPEED:
                    w = np.sign(w) * self.opt.MAX_ANGULAR_SPEED
            else:
                # 已到达目标点，导航结束
                self.Is_arrive = True
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
        """
        根据多个路径点生成完整路径
        
        该函数通过连接相邻路径点之间的路径段，构建一条经过所有指定点的完整路径。
        同时设置路径跟踪器和障碍物检测器的相关路径信息。
        
        参数:
            point_list: 路径点列表，每个元素应包含路径点的坐标信息
            
        返回:
            list: 包含所有有效路径段端点位置的列表
        """
        pathw_m = None
        positions = []
        # 如果路径点列表有效且包含至少两个点，则进行路径规划
        if point_list is not None and len(point_list) > 1:
            # 遍历相邻的路径点对，依次规划路径段
            for ps, pe in zip(point_list[:-1], point_list[1:]):
                _, pw, pos_start, pos_end = self.make_route(ps, pe)
                # 检查路径段是否有效，如果有效则连接到总路径中
                if is_pathw_valid(pw):
                    pathw_m = pathw_concatenation(pathw_m, pw)
                    # 记录路径段的起始和结束位置
                    if len(positions) == 0:
                        positions.append(pos_start)
                    positions.append(pos_end)

        # 计算路径上各点的偏航角
        pathw_m.calc_yaw_np()
        self.pathw = pathw_m

        # 设置路径跟踪器和障碍物检测器的路径信息
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
