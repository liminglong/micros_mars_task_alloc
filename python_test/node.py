#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy
from std_msgs.msg import String

def avoid_headingCB():
    print 'hello'

rospy.init_node('python_test')
topic_name = rospy.get_param('topic_name', 'default_topic_name')
sub = rospy.Subscriber(topic_name, String, avoid_headingCB)
rospy.spin()
    
