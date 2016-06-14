#!/usr/bin/env python

__author__ = 'Minglong Li'

from robot_patrol import RobotPatrol

class RobotPatrolArea1(RobotPatrol):
    def __init__(self, topic = None, node_name = None):
        self.pose_0 = [4.0, 6.0, 0.0, 1.0]
        self.pose_1 = [5.9, 6.05, 0.706, 0.709]
        self.pose_2 = [5.9, 8.81, 1.0, 0.0]
        self.pose_3 = [4.0, 8.88, -0.707, 0.707]
        if node_name is None:
            node_name = 'robot_patrol_area_1'
        if topic is None:
            topic = 'robot_1/move_base/switch' 
        self._node_name = node_name 
        self._topic = topic
        RobotPatrol.__init__(self, node_name = self._node_name, pose_0 = self.pose_0, pose_1 = self.pose_1, pose_2 = self.pose_2, pose_3 = self.pose_3, topic = self._topic, wait_time = 6.0)

    def run(self):
        RobotPatrol.start_patrol(self)
