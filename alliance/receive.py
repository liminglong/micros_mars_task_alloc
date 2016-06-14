#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import rospy

class Receive(object):
    def __init__(self, topic, msg_type):
        self._topic = topic
        self._msg_type = msg_type
        sub = rospy.Subscriber(self._topic, self._msg_type, self.on_received())

    def on_received(self, msg):
        pass
