#!/usr/bin/env python3
# encoding: utf-8

"""
文件名: central_manager.py
简介： navigation导航节点
作者： 未定义实验室.Zean 罗灵轩
版本： 1.0.0
说明： navigation导航节点
更新内容： 第一个稳定版本
创建时间： 2025.8.5
最后更新时间： 2025.8.25
"""

import queue
from queue import Queue, SimpleQueue
from pyadroute.utils.logger import get_logger
import ros_helper as roser
import rospy
import threading

# import pyroutelib3
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from turn_on_wheeltec_robot.msg import MasterCmd, TrafficSignal
from visualization_msgs.msg import Marker, MarkerArray
from pyadroute.ad_navigation_adaptor import AdNavigationAdapter
from pyadroute.utils.path import euclidean_distance, distance_of_two_point_ahead_path

import numpy as np
from param_helper import ParamHelper


from std_msgs.msg import String

logger = get_logger("NavigationNode")


class NavigationNode:
    tracked_pose: PoseStamped
    navigation: AdNavigationAdapter
    cmd: MasterCmd
    last_cmd: MasterCmd

    def __init__(self, name):

        self.cmd_pause = False
        self.arrive_jishu = False

        rospy.init_node(name)

        self.opt = ParamHelper()

        self.loop_rate = rospy.get_param("~loop_rate", 100)
        osm_filename = rospy.get_param("~osm_filename", "")

        opt = AdNavigationAdapter.Option()
        opt.Kp_rho = self.opt.Kp_rho
        opt.Kp_alpha = self.opt.Kp_alpha
        opt.Kd_alpha = self.opt.Kd_alpha
        opt.Kp_beta = self.opt.Kp_beta
        opt.obstacle_dist = self.opt.obstacle_dist
        opt.MAX_LINEAR_SPEED = self.opt.MAX_LINEAR_SPEED
        opt.MAX_ANGULAR_SPEED = self.opt.MAX_ANGULAR_SPEED
        opt.LOOK_AHEAD_DISTANCE = self.opt.LOOK_AHEAD_DISTANCE
        opt.Refine_Path_Step = self.opt.Refine_Path_Step
        

        self.navigation = AdNavigationAdapter(osm_filename, opt)
        print(self.navigation.route_adaptor.G.graph)

        self.osm_msg = roser.create_osm_ros_MarkerArray2(
            self.navigation.route_adaptor.G
        )

        self.other_car_pose = None
        self.tracked_pose = None
        self.target_queue = SimpleQueue()
        self.master_cmd = None
        self.last_cmd = None

        self.double_car_pursuit = False

        self.pub_osm = rospy.Publisher("/graph_osm", MarkerArray, queue_size=1)

        self.pub_path = rospy.Publisher("/path", Path, queue_size=1)
        self.pub_point = rospy.Publisher("/path_endpoint", MarkerArray, queue_size=1)
        self.pub_obstacle_area = rospy.Publisher("/obstacle_area", Marker, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        

        rospy.Subscriber("/nav_cmd",String, self.nav_cmd_callback,queue_size=10)
        rospy.Subscriber("/nav_goal", PointStamped, self.nav_goal_callback,queue_size=10)
        self.result = rospy.Publisher("/result", String, queue_size=1)


        self.sub_cloudpoint = rospy.Subscriber(
            self.opt.TOPIC_CLOUD_POINT, PointCloud2, self.callback_cloudpoint
        )
        self.sub_tracked_pose = rospy.Subscriber(
            self.opt.TOPIC_TRACKED_POSE, PoseStamped, self.callback_tracked_pose
        )
        self.sub_rviz = rospy.Subscriber(
            "/clicked_point", PointStamped, self.callback_rviz
        )
        self.sub_mastercmd = rospy.Subscriber(
            self.opt.TOPIC_MASTER_CMD,
            MasterCmd,
            self.callback_mastercmd,
            queue_size=10,
        )
        self.sub_other_car = rospy.Subscriber(
            self.opt.TOPIC_OTHER_CAR_POSE, PoseStamped, self.callback_othercar
        )

        self.sub_traffic_sig = rospy.Subscriber(
            self.opt.TOPIC_TRAFFIC_SIGNAL, TrafficSignal, self.callback_traffic_signal
        )

    def nav_cmd_callback(self, msg):
        rospy.logerr(": %s" % msg.data)
        if msg.data == "/test_forward":
            self.cmd_pause = True
            self.publish_cmd_vel(0.3,0.0)
            rospy.sleep(1)
            self.cmd_pause = False
        elif msg.data == "/test_backward":
            self.cmd_pause = True
            self.publish_cmd_vel(-0.3,0.0)
            rospy.sleep(1)
            self.cmd_pause = False
        elif msg.data == "/test_left":
            self.cmd_pause = True
            self.publish_cmd_vel(0.3,0.4)
            rospy.sleep(1)
            self.cmd_pause = False
        elif msg.data == "/test_right":
            self.cmd_pause = True
            self.publish_cmd_vel(0.3,-0.4)
            rospy.sleep(1)
            self.cmd_pause = False

        if msg.data == "/pause":
            rospy.logerr(": pause")
            self.cmd_pause = True
        elif msg.data == "/continue":
            rospy.logerr(": continue")
            self.cmd_pause = False
        elif msg.data == "/stop":
            rospy.logerr(": stop")
            while True:
                try:
                    _ = self.target_queue.get_nowait()
                except queue.Empty:
                    break
            self.navigation.clear_path()


    def nav_goal_callback(self, msg: PointStamped):
        try:
            self.cmd_pause = True
            self.navigation.clear_path()
            while True:
                try:
                    _ = self.target_queue.get_nowait()
                except queue.Empty:
                    break
            self.target_queue.put_nowait((msg.point.x, msg.point.y))
            self.cmd_pause = False
            logger.warning("收到: %s", str((msg.point.x, msg.point.y)))
        except queue.Full:
            logger.error("已经塞满了")
        except AttributeError:
            logger.error("参数错误")




    def run(self):
        pub_osm_thread = threading.Thread(target=self.thread_publish_osm)
        pub_osm_thread.start()

        rospy.loginfo("------Navigation Node start-------")

        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            #测试以下，只有当没有收到暂停指令时才运行
            if self.cmd_pause == False:
                self.process_loop()
            rate.sleep()

        pub_osm_thread.join()
        rospy.loginfo("------Navigation Node stop-------")

    def process_loop(self):
        """
        处理导航循环的主要逻辑函数
        
        该函数负责从目标队列中获取目标点，生成路径，并根据当前跟踪位姿计算速度指令。
        主要包括路径规划、速度控制和指令发布等功能。
        
        参数:
            无
            
        返回值:
            无
        """
        target_point = None
        # 如果当前没有路径，则尝试从目标队列中获取新的目标点
        if self.navigation.pathw is None:
            try:
                target_point = self.target_queue.get_nowait()
                logger.error("取出一个")
            except queue.Empty:
                pass

        # 执行主控制逻辑
        self.master_cmd_logic()
        tracked_pose = self.tracked_pose  # thread safe

        if tracked_pose is not None:
            # 如果没有路径且成功获取到目标点，则创建新路径
            if self.navigation.pathw is None and target_point is not None:
                point_start = (
                    tracked_pose.pose.position.x,
                    tracked_pose.pose.position.y,
                )
                self.make_path([point_start, target_point])

            # 处理导航循环，计算距离、线速度、角速度等信息
            (
                distance,
                linear_speed,
                angle_speed,
                nearest_point_index,
                point_target_idx,
            ) = self.navigation.process_loop(tracked_pose.pose)
            
            # 如果已到达目标点，发布到达消息
            if self.navigation.Is_arrive:
                logger.error("到了")
                # self.nav_cmd_callback(String("/stop"))
                self.result.publish(String("/arrive"))
                self.navigation.Is_arrive = False
                
            # 根据车辆距离控制调整速度
            linear_speed, angle_speed = self.control_dbl_car_distance(
                linear_speed, angle_speed, nearest_point_index
            )
            
            # 发布速度控制指令
            #logger.error("发布速度控制指令")
            self.publish_cmd_vel(linear_speed, angle_speed)
        else:
            # 如果没有跟踪到位姿信息，则发布零速度指令
            self.publish_cmd_vel(0, 0)

    def make_path(self, point_list):
        """
        根据给定的点列表生成导航路径
        
        :param point_list: 点坐标列表，用于生成路径的途经点
        :return: 无返回值
        
        该函数首先检查点列表是否有效，然后调用导航模块生成多点路径，
        如果路径有效则发布路径信息，否则记录错误日志并清理路径数据。
        """
        # 检查点列表是否有效（非空且至少包含两个点）
        if point_list is not None and len(point_list) > 1:
            try:
                # 调用导航模块生成多点路径
                positions = self.navigation.make_route_muti_point(point_list)
                
                # 验证生成的路径是否有效
                if self.navigation.is_path_valid():
                    # 发布路径信息和路径端点信息
                    self.publish_path(self.navigation.pathw.path)
                    self.publish_path_endpoint(positions)
                    logger.warning("make path 成功")
                else:
                    # 路径无效时记录错误日志并清理路径数据
                    logger.error("无法到达目标点，make path失败")
                    self.navigation.clear_path()
            except Exception as e:
                # 异常处理：记录错误日志并清理路径数据
                logger.error(f"make path 失败：{e}")
                self.navigation.clear_path()

    def callback_cloudpoint(self, msg):
        self.navigation.obstacle_detector.set_pointcloud(msg)

    def callback_tracked_pose(self, msg: PoseStamped):
        self.tracked_pose = msg  # thread safe
        # logger.info("tracked_pose: %s", str(msg))

    def callback_rviz(self, msg: PointStamped):
        try:
            self.target_queue.put_nowait((msg.point.x, msg.point.y))
            logger.info("Clicked point: %s", str((msg.point.x, msg.point.y)))
        except queue.Full:
            rospy.logerr("target_queue was full.")
        except AttributeError:
            logger.error("不是吧老弟，又是空的？")

    def callback_mastercmd(self, msg: MasterCmd):
        self.set_cmd(msg)

    def callback_othercar(self, msg: PoseStamped):
        self.other_car_pose = msg

    def callback_traffic_signal(self, msg: TrafficSignal):
        self.navigation.set_traffic_signal(msg)

    def thread_publish_osm(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.osm_msg is not None:
                self.pub_osm.publish(self.osm_msg)

            self.public_obstacle_area()

            rate.sleep()

    def publish_path(self, path):
        msg = roser.create_path_msg(path)
        if msg is not None:
            self.pub_path.publish(msg)

    def publish_path_endpoint(self, positions):
        """
        发布路径端点标记到ROS话题
        
        该函数将位置信息转换为MarkerArray消息并发布到指定的话题中，
        用于在ROS环境中可视化路径的端点位置。
        
        参数:
            positions: 路径端点的位置信息，具体格式取决于roser.create_path_endpoint_MarkerArray的实现
            
        返回值:
            无返回值
            
        发布:
            将生成的MarkerArray消息发布到self.pub_point话题
        """
        msg = roser.create_path_endpoint_MarkerArray(positions)
        # 如果消息创建成功，则发布消息
        if msg is not None:
            self.pub_point.publish(msg)

    def public_obstacle_area(self):
        """
        发布障碍物检测区域信息
        
        该函数获取当前导航模块中的障碍物检测区域坐标，将其转换为ROS消息格式，
        并通过指定的话题发布出去。
        
        参数:
            无
            
        返回值:
            无
        """
        # 获取障碍物检测区域的坐标信息
        detect_area = self.navigation.obstacle_detector.get_detect_area_coords()
        
        # 创建线条消息用于可视化检测区域
        msg = roser.create_line_strip_msg(detect_area)
        
        # 如果消息创建成功，则发布消息
        if msg is not None:
            self.pub_obstacle_area.publish(msg)

    def publish_cmd_vel(self, linear_speed, angle_speed):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angle_speed

        self.pub_cmd_vel.publish(msg)

    def master_cmd_logic(self):
        """
        处理主控命令的逻辑，根据命令内容生成对应的路径规划。
        
        该函数会根据主控命令的内容判断是否需要进行编队行驶或普通路径规划，
        并调用make_path方法生成相应的路径。处理完成后会清空主控命令。

        Returns:
            bool: 如果成功处理了命令并生成了路径则返回True，否则返回False
        """
        cmd: MasterCmd = self.master_cmd
        if cmd is None:
            return

        finished = False
        self.double_car_pursuit = False
        
        # 处理停止命令
        if cmd.start == 0:
            self.navigation.clear_path()
            finished = True
        else:
            # 判断是否为编队行驶模式
            if cmd.cid != cmd.set_leader and cmd.forma > 0:
                # logger.warning("编队行驶模式")
                # Formation driving
                other_car_pose = self.other_car_pose
                tracked_pose = self.tracked_pose  # thread safe
                if other_car_pose is not None and tracked_pose is not None:
                    # 构造四点路径：当前位姿 -> 其他车辆位姿 -> 起始点 -> 终点
                    p0 = (
                        tracked_pose.pose.position.x,
                        tracked_pose.pose.position.y,
                    )
                    p1 = (
                        other_car_pose.pose.position.x,
                        other_car_pose.pose.position.y,
                    )
                    # p2 = (cmd.start_rp.position.x, cmd.start_rp.position.y)
                    p3 = (cmd.end_rp.position.x, cmd.end_rp.position.y)
                    # logger.info("master cmd p1 p2: %s,%s", p1, p2)
                    # self.make_path([p0, p1, p2, p3])
                    self.make_path([p0, p1, p3])
                    self.double_car_pursuit = True
                    finished = True
            else:
                # 普通路径规划模式
                # logger.warning("普通路径规划模式")
                tracked_pose = self.tracked_pose  # thread safe
                if tracked_pose is not None:
                    # 构造三点路径：当前位姿 -> 起始点 -> 终点
                    p0 = (
                        tracked_pose.pose.position.x,
                        tracked_pose.pose.position.y,
                    )
                    # p1 = (cmd.start_rp.position.x, cmd.start_rp.position.y)
                    p2 = (cmd.end_rp.position.x, cmd.end_rp.position.y)
                    # logger.info("master cmd p1 p2: %s,%s", p1, p2)
                    # self.make_path([p0, p1, p2])
                    self.make_path([p0, p2])
                    finished = True
        # self.nav_cmd_callback(String("/continue"))
        if self.cmd_pause == True:
            self.cmd_pause = False
        # 命令处理完成后清空主控命令
        if finished:
            self.master_cmd = None

        return finished

    def set_cmd(self, cmd):
        """
        设置命令并判断是否为新命令
        
        该函数用于设置新的命令，并通过比较命令的关键属性来判断是否为新命令。
        如果是新命令，则更新master_cmd和last_cmd属性。
        
        参数:
            cmd: 命令对象，包含命令的相关属性
            
        返回值:
            bool: 如果是新命令返回True，否则返回False
        """
        is_new = False
        logger.info(cmd)
        if cmd is not None:
            # 判断是否为新命令：如果last_cmd为空，则为新命令
            if self.last_cmd is None:
                is_new = True
            else:
                # 比较命令的关键属性，任一属性不同则认为是新命令
                if cmd.cid != self.last_cmd.cid:
                    is_new = True
                elif cmd.forma != self.last_cmd.forma:
                    is_new = True
                elif cmd.start_rp != self.last_cmd.start_rp:
                    is_new = True
                elif cmd.end_rp != self.last_cmd.end_rp:
                    is_new = True
                elif cmd.start != self.last_cmd.start:
                    is_new = True
                elif cmd.set_leader != self.last_cmd.set_leader:
                    is_new = True
        # 如果是新命令，则更新master_cmd和last_cmd
        if is_new:
            self.master_cmd = cmd
            self.last_cmd = cmd
            #self.nav_cmd_callback(String("/continue")) #收到命令后，发送/continue命令
        if self.cmd_pause == True:
            self.cmd_pause = False
        return is_new

    def control_dbl_car_distance(self, linear_speed, angle_speed, orin_index):
        """
        控制双车距离的函数，用于调整当前车辆与前车的距离保持在安全范围内
        
        参数:
            linear_speed: 当前线速度
            angle_speed: 当前角速度
            orin_index: 路径上的索引位置
            
        返回值:
            tuple: (调整后的线速度, 调整后的角速度)
        """
        if (
            self.double_car_pursuit
            and abs(linear_speed) > 0
            and orin_index >= 0
            and self.navigation.is_path_valid()
        ):
            # 获取前车位置信息
            other_car_pose = self.other_car_pose
            if other_car_pose is not None:
                p1 = (
                    other_car_pose.pose.position.x,
                    other_car_pose.pose.position.y,
                )

                # 计算当前车辆与前车在路径上的距离
                distance, dest_idx = distance_of_two_point_ahead_path(
                    self.navigation.pathw.path, orin_index, p1
                )
                
                # 根据距离判断是否需要调整速度
                if distance < self.opt.DBL_CAR_STOP_DIST:
                    # 距离过近，停止行驶
                    linear_speed = 0
                    # angle_speed = 0
                elif distance > 0:
                    # 距离适中，进行跟随控制
                    dist_diff = distance - self.opt.DBL_CAR_FOLLOW_DIST
                    Kp_dist = 0.5
                    linear_speed = linear_speed + Kp_dist * dist_diff

                    # 限制最大线速度
                    if linear_speed > self.opt.MAX_LINEAR_SPEED:
                        linear_speed = self.opt.MAX_LINEAR_SPEED

                logger.info("double car orin_index=%d p1 %d dist:%.2f speed:%.2f" % (orin_index, dest_idx, distance, linear_speed))

        return linear_speed, angle_speed


if __name__ == "__main__":
    node = NavigationNode("Navigation_Node")
    node.run()
    # try:
    #     node.run()
    # except Exception as e:
    #     logger.exception(e)
