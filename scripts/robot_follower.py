#!/usr/bin/env python
# coding=utf-8
'''
跟踪的检测节点怎么写：
   1. 如果不同机器人的检测节点检测到了可疑机器人进入到了自己的探测区域，那么就会更新parameter server中和tf发布有关的参数;
   2. 相应的跟踪机器人会更新parameter server中的跟tf有关的参数，这样的话，跟踪就会启动;
   3. 跟踪发出的速度消息通过一个suppressor去抑制下层的navigation发出的消息;
   4. 当可疑机器人逃逸，跟踪失败之后，当前跟踪机器人会继续之前的巡逻任务。

'''

__author__ = 'Minglong Li'

import rospy
import math
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class Robot_follower(object):
    def __init__(self, name):
        self.listener = tf.TransformListener()
    def robot_listener(self):
        '''
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(4, 2, 0, 'turtle2')
        '''
        robot_vel_pub = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/robot_3', '/robot_0', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            robot_vel_pub.publish(cmd)
    
            rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('robot_follower')
    robot_follower =  Robot_follower(rospy.get_name())
    robot_follower.robot_listener()
    rospy.spin()

