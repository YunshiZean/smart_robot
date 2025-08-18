import math
from .utils.angle import quaternion_to_yaw
from geometry_msgs.msg import Pose

class State:

    def __init__(self, pose: Pose):
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = quaternion_to_yaw(pose.orientation)

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)