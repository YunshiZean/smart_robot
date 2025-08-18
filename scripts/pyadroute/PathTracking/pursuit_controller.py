"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""

import numpy as np
import math

from ..path_wraper import PathWrapper

from geometry_msgs.msg import Pose
from ..state import State


class PursuitController:

    def __init__(self, look_ahead_distance):
        self.Lfc = look_ahead_distance
        self.cx = None
        self.cy = None
        self.path_yaw = None
        self.old_nearest_point_index = None
        self.old_target_point_index = None

    def set_pathw(self, pathw: PathWrapper):
        if pathw is None or pathw.length() == 0:
            self.cx = None
            self.cy = None
            self.path_yaw = None
        else:
            self.cx, self.cy = zip(*pathw.path)
            self.path_yaw = pathw.yaws

        self.old_nearest_point_index = None
        self.old_target_point_index = None

    def search_target_index(self, state: State):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index

            if ind < len(self.cx) - 1:
                distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
                for ii in range(ind + 1, len(self.cx)):
                    distance_next_index = state.calc_distance(self.cx[ii], self.cy[ii])
                    if distance_this_index < distance_next_index:
                        break
                    ind = ii
                    distance_this_index = distance_next_index
            else:
                ind = len(self.cx) - 1

            self.old_nearest_point_index = ind

        # search Lfc look ahead target point index
        if ind < len(self.cx) - 1:
            for ii in range(ind + 1, len(self.cx)):
                distance_next_index = state.calc_distance(self.cx[ii], self.cy[ii])
                if distance_next_index <= self.Lfc:
                    ind = ii
                else:
                    break

        Lf = state.calc_distance(self.cx[ind], self.cy[ind])

        if self.old_target_point_index is None:
            self.old_target_point_index = ind
        elif self.old_target_point_index > ind:
            ind = self.old_target_point_index
        else:
            self.old_target_point_index = ind

        return self.old_nearest_point_index, ind, Lf

    def pursuit_control(self, state: State):

        nearest_point_index = target_point_idx = Lf = None
        
        if self.cx is not None:
            nearest_point_index, target_point_idx, Lf = self.search_target_index(state)

        # tx = self.cx[ind]
        # ty = self.cy[ind]

        # steering_angle = math.atan2(ty - state.y, tx - state.x) + self.path_yaw[ind] - state.yaw

        return nearest_point_index, target_point_idx, Lf
