"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
from random import random
from ..utils.angle import angle_mod


class PoseController:
    """
    Constructs an instantiate of the PathFinderController for navigating a
    3-DOF wheeled robot on a 2D plane

    Parameters
    ----------
    Kp_rho : The linear velocity gain to translate the robot along a line
             towards the goal
    Kp_alpha : The angular velocity gain to rotate the robot towards the goal
    Kp_beta : The offset angular velocity gain accounting for smooth merging to
              the goal angle (i.e., it helps the robot heading to be parallel
              to the target angle.)
    """

    Kp_rho: float
    Kp_alpha: float
    Kp_beta: float
    Kd_alpha: float

    def __init__(self, Kp_rho, Kp_alpha, Kd_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kd_alpha = Kd_alpha
        self.Kp_beta = Kp_beta
        self.last_alpha = 0

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        # print(rho)
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        # beta = angle_mod(theta_goal - theta)
        # beta = 0
        v = self.Kp_rho * rho
        # w = self.Kp_alpha * alpha - self.Kp_beta * beta       
        alpha_diff =  alpha-self.last_alpha
        w = self.Kp_alpha * alpha + self.Kp_beta * beta - self.Kd_alpha*alpha_diff

        self.last_alpha = alpha

        # print("diff=%.2f theta=%.2f theta_goal=%.2f alpha=%.2f alpha_diff=%.2f beta=%.2f w=%.2f v=%.2f" 
        #       % (np.arctan2(y_diff, x_diff),theta,theta_goal,alpha,alpha_diff,beta,w,v))

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            # v = -v
            v = 0.0
            w = 0.0 
            # TODO maybe stop             

        return rho, v, w
