#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy
import math

import geometry_msgs.msg
from nav_msgs.msg import Odometry

class Robot_follow(object):


    def __init__(self, name):
        #self.follower = rospy.get_param("~follower")
        #self.intruder = rospy.get_param("~intruder")
        self.follower_pose_x = 0.0
        self.follower_pose_y = 0.0
        self.intruder_pose_x = 0.0
        self.intruder_pose_y = 0.0 
        self.follow_distance = 0.5 # The follower will be away from the intruder according to this distance      
        self.linear_velocity = 0.3
        self.angular_velocity = 0.5
        self.follower = 'robot_0'
        self.intruder = 'robot_3'
        self.sub1 = rospy.Subscriber('/%s/base_pose_ground_truth' % self.follower,
                                    Odometry,
                                    self.robot_follow_cb)
        self.sub2 = rospy.Subscriber('/%s/base_pose_ground_truth' % self.intruder,
                                    Odometry,
                                    self.intruder_pose_update_cb)        

        self.robot_vel_pub = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)


    def robot_follow_cb(self, msg):

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w 
        print "self.intruder_pose_y, self.follower_pose_y, self.intruder_pose_x, self.follower_pose_x: %f %f %f %f" %(self.intruder_pose_y, self.follower_pose_y, self.intruder_pose_x, self.follower_pose_x)
        print "2*(w*z+x*y), 1-2*(y**2+z**2): %f %f" %( 2*(w*z+x*y), 1-2*(y**2+z**2) )
        print "self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x: %f %f" %(self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x)
        leader_direction = math.atan2(self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x)
        if (leader_direction > -1*math.pi or leader_direction == -1*math.pi) and leader_direction < 0:
            leader_direction = 2*math.pi + leader_direction

        follower_orientation = math.atan2(2*(w*z+x*y), 1-2*(y**2+z**2))
        if (follower_orientation > -1*math.pi or follower_orientation  == -1*math.pi) and follower_orientation < 0:
            follower_orientation = 2*math.pi + follower_orientation
        print 'follower_orientation: %f' %(follower_orientation)
        print 'leader_direction: %f' %(leader_direction)


        self.follower_pose_x = msg.pose.pose.position.x
        self.follower_pose_y = msg.pose.pose.position.y
        if ((self.follower_pose_x - self.intruder_pose_x)**2 + (self.follower_pose_y - self.intruder_pose_y)**2) > self.follow_distance ** 2:
            if follower_orientation < leader_direction:  
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = self.angular_velocity
                self.robot_vel_pub.publish(cmd)
            else:
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = -1 * self.angular_velocity
                self.robot_vel_pub.publish(cmd)
        else:
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.robot_vel_pub.publish(cmd)
                        
    def intruder_pose_update_cb(self, msg):
        self.intruder_pose_x = msg.pose.pose.position.x
        self.intruder_pose_y = msg.pose.pose.position.y
     
if __name__ == '__main__':
    rospy.init_node('robot_follow')
    robot_follow_obj = Robot_follow(rospy.get_name())
    rospy.spin()



