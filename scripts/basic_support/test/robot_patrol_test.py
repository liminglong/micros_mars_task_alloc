#!/usr/bin/env python

__author__ = 'Minglong Li'

import sys
sys.path.append("~/catkin_ws/src/multi_robot_patrol/scripts/basic_support")

from robot_patrol_area_0 import RobotPatrolArea0
from robot_patrol_area_1 import RobotPatrolArea1
from robot_patrol_area_2 import RobotPatrolArea2

ob1 = RobotPatrolArea0()
ob2 = RobotPatrolArea1()
ob3 = RobotPatrolArea2()
ob1.start()
ob2.start()
ob3.start()


