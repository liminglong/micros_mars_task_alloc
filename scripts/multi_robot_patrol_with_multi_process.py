#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import multiprocessing
import rospy
import robot_patrol_area0
import robot_patrol_area1
import robot_patrol_area2

class Multi_robot_patrol(object):
    def __init__(self):
        self.robot0_patrol = robot_patrol_area0.Robot_patrol()
        self.robot1_patrol = robot_patrol_area1.Robot_patrol()
        self.robot2_patrol = robot_patrol_area2.Robot_patrol()

    def start_multi_robot_patrol(self):
        self.robot0_patrol.start()
        self.robot1_patrol.start()
        self.robot2_patrol.start()

if __name__ == '__main__':
    multi_robot_patrol = Multi_robot_patrol()
    multi_robot_patrol.start_multi_robot_patrol()
