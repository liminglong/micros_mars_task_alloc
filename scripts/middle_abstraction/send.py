#!/usr/bin/env python 
# coding=utf-8

__author__ = "Minglong Li"

import rospy

class Send(object):
    def __init__(self, topic, msg_type):
        self._topic = topic
        self._msg_type = msg_type
        self.pub = rospy.Publisher(self._topic, self._msg_type, queue_size = 1)

    def send(self, msg):
        self.pub.publish(msg)
