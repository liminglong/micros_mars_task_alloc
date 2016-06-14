#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy
import multiprocessing
import actionlib

import multi_robot_patrol.msg

class Robot_patrol(multiprocessing.Process):
    def __init__(self):
        multiprocessing.Process.__init__(self)

    def run(self):
        rospy.init_node('robot_patrol_area1') 
        print 'Patroller 1 starts!'
        client = actionlib.SimpleActionClient('robot_1/move_base', multi_robot_patrol.msg.MoveBaseAction)
        client.wait_for_server()

        #Several goals to be sent to the action server.
        goal1 = multi_robot_patrol.msg.MoveBaseGoal()
        goal1.target_pose.header.frame_id = 'map'
        goal1.target_pose.pose.position.x = 4.2
        goal1.target_pose.pose.position.y = 6.0
        goal1.target_pose.pose.position.z = 0.0
        goal1.target_pose.pose.orientation.x = 0.0
        goal1.target_pose.pose.orientation.y = 0.0
        goal1.target_pose.pose.orientation.z = 0.0
        goal1.target_pose.pose.orientation.w = 1	

        goal2 = multi_robot_patrol.msg.MoveBaseGoal()
        goal2.target_pose.header.frame_id = 'map'
        goal2.target_pose.pose.position.x = 5.9
        goal2.target_pose.pose.position.y = 6.05
        goal2.target_pose.pose.position.z = 0.0
        goal2.target_pose.pose.orientation.x = 0.0
        goal2.target_pose.pose.orientation.y = 0.0
        goal2.target_pose.pose.orientation.z = 0.706
        goal2.target_pose.pose.orientation.w = 0.709

        goal3 = multi_robot_patrol.msg.MoveBaseGoal()
        goal3.target_pose.header.frame_id = 'map'
        goal3.target_pose.pose.position.x = 5.9
        goal3.target_pose.pose.position.y = 8.81
        goal3.target_pose.pose.position.z =  0.0
        goal3.target_pose.pose.orientation.x = 0.0
        goal3.target_pose.pose.orientation.y = 0.0
        goal3.target_pose.pose.orientation.z = 1.0
        goal3.target_pose.pose.orientation.w = 0.0 	

        goal4 = multi_robot_patrol.msg.MoveBaseGoal()
        goal4.target_pose.header.frame_id = 'map'
        goal4.target_pose.pose.position.x = 4.06
        goal4.target_pose.pose.position.y = 8.88
        goal4.target_pose.pose.position.z = 0.0
        goal4.target_pose.pose.orientation.x = 0.0
        goal4.target_pose.pose.orientation.y = 0.0
        goal4.target_pose.pose.orientation.z = -0.707
        goal4.target_pose.pose.orientation.w = 0.707

        while not rospy.is_shutdown():
            client.send_goal(goal1)
            client.wait_for_result(rospy.Duration.from_sec(6.0))

            client.send_goal(goal2)
            client.wait_for_result(rospy.Duration.from_sec(6.0))
            
            client.send_goal(goal3)
            client.wait_for_result(rospy.Duration.from_sec(6.0))
            
            client.send_goal(goal4)
            client.wait_for_result(rospy.Duration.from_sec(6.0))
'''
if __name__ == '__main__':
    robot_patrol_obj = Robot_patrol(rospy.get_name())
    robot_patrol_obj.patrol_client()
'''


