#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Robot_detect(object):
    br = CvBridge()
    count = 0
    def __init__(self, name):
        sub = rospy.Subscriber("image", Image, self.image_cb)
    def image_cb(self, msg):
        im = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #print im
        for i in range(0, im.shape[0]):
            for j in range(0, im.shape[1]):
                if im[i, j, 0] == 255 and im[i, j, 1] == 0 and im[i, j, 2] == 0:
                    print 'Detect an intruder!' 

if __name__ == '__main__':
    rospy.init_node('robot_detect')
    robot_detect = Robot_detect(rospy.get_name())
    rospy.spin()



