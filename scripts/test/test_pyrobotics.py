#!/usr/bin/env python3
# encoding: utf-8

import sys
pkg_rootdir=r'/home/ros/ros_ws/smart_robot_ws/src/autonomous_driving/scripts/PythonRobotics'
if pkg_rootdir not in sys.path:
    sys.path.append(pkg_rootdir)
print(sys.path)