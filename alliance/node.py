#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy
import multiprocessing

class Node(multiprocessing.Process):
    def __init__(self):
        multiprocessing.Process.__init__(self)
