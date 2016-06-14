#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import rospy

class Receive(object):
    def __init__(self, topic, msg_type, on_received):
        self._topic = topic
        self._msg_type = msg_type
        self._on_received = on_received
        sub = rospy.Subscriber(self._topic, self._msg_type, self._on_received)

