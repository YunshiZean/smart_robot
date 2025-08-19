# coding=utf-8
import rospy
import numpy as np

class ParamHelper:
    
    DBL_CAR_FOLLOW_DIST = 0.8 #双车追逐最小间距
    DBL_CAR_STOP_DIST = 0.5 #双车追逐停车间距

    MAX_LINEAR_SPEED = 0.35
    MAX_ANGULAR_SPEED = 0.5#np.radians(90.0)

    LOOK_AHEAD_DISTANCE = 0.25

    Refine_Path_Step = 0.01

    # PoseController
    Kp_rho = 2.0
    Kp_alpha = 0.6
    Kd_alpha = 0.0 #0.5
    Kp_beta = 0.15
    obstacle_dist = 0.3 #障碍检测距离

    def __init__(self):
        self.TOPIC_OTHER_CAR_POSE = rospy.get_param(
            "TOPIC_OTHER_CAR_POSE", "/other_car/pose"
        )  # 发布其他car位姿
        self.TOPIC_TRAFFIC_SIGNAL = rospy.get_param(
            "TOPIC_TRAFFIC_SIGNAL", "/traffic_signal"
        )  # 发布交通灯
        self.TOPIC_MASTER_CMD = rospy.get_param(
            "TOPIC_MASTER_CMD", "/master_cmd"
        )  # 转发上位机指令

        self.TOPIC_VEHICLE_STATUS = rospy.get_param(
            "TOPIC_VEHICLE_STATUS", "/vehicle_status"
        )  # 当前car状态
        self.TOPIC_TRACKED_POSE = rospy.get_param(
            "TOPIC_TRACKED_POSE", "/tracked_pose"
        )  # 当前car位姿

        self.TOPIC_CLOUD_POINT = rospy.get_param(
            "TOPIC_CLOUD_POINT", "/scan_matched_points2"
        )  # 当前car位姿

