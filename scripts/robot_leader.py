#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy

import tf
import math

import geometry_msgs.msg
from nav_msgs.msg import Odometry

class Robot_leader(object):
    def __init__(self, name):
        self.robotname = rospy.get_param("~robot")
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber('/%s/base_pose_ground_truth' % self.robotname,
                                    Odometry,
                                    self.handle_robot_pose, 
                                    self.robotname)
                      
    def handle_robot_pose(self, msg, robotname):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w 
        yaw =  math.atan2(2*(w*z+x*y), 1-2*(y**2+z**2))
        if yaw > -1 * math.pi and yaw < 0:
            yaw = 2 * math.pi + yaw
        print "yaw: %f" %(yaw)
        self.br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                               tf.transformations.quaternion_from_euler(0, 0, yaw),
                               rospy.Time.now(),
                               self.robotname,
                               "map")#TODO:msg.theta
    
if __name__ == '__main__':
    rospy.init_node('robot_leader')
    robot_leader = Robot_leader(rospy.get_name())
    rospy.spin()

