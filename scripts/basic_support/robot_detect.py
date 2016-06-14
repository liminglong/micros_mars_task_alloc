#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import sys
sys.path.append("..")
from middle_abstraction.function_unit import FunctionUnit
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

'''
class Robot_detect(object):
    br = CvBridge()
    count = 0
    def __init__(self, name):
        sub = rospy.Subscriber("robot_0/image", Image, self.image_cb)
    def image_cb(self, msg):
        im = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        print 'message received'
        for i in range(0, im.shape[0]):
            for j in range(0, im.shape[1]):
                if im[i, j, 0] == 255 and im[i, j, 1] == 0 and im[i, j, 2] == 0:
                    print 'Detect an intruder!' 

if __name__ == '__main__':
    rospy.init_node('robot_detect')
    robot_detect = Robot_detect(rospy.get_name())
    rospy.spin()
'''

class RobotDetect(FunctionUnit):
    def __init__(self, node_name, receive_topic, send_topic, virtual_off= None):
        FunctionUnit.__init__(self, node_name)    
        self._receive_topic = receive_topic
        self._send_topic = send_topic
        self._virtual = virtual_off
        self.br = CvBridge()
        self._virtual_send = FunctionUnit.create_send(self, virtual_off, Bool)
        
    def run(self):
        pass#这里也可以改成子类直接调用父类的run方法来做。

    def start_detect(self):
        FunctionUnit.init_node(self)
        #print 'hello 1'
        #print self._receive_topic
        receive_1 = FunctionUnit.create_receive(self, self._receive_topic, Image, self.receive_1_cb)
        #print 'hello 2'
        FunctionUnit.spin(self)
    
    def receive_1_cb(self, msg):
        #print 'message received'
        #print msg
        im = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        for i in range(0, im.shape[0]):
            for j in range(0, im.shape[1]):
                #if not (im[i, j, 0] == 255 and im[i, j, 1] == 0 and im[i, j, 2] == 0):
                if im[i, j, 0] == 255 and im[i, j, 1] == 0 and im[i, j, 2] == 0:
                    #print 'Detect an intruder!'      
                    msg_sended = Bool()
                    msg_sended.data = True             
                    send = FunctionUnit.create_send(self, self._send_topic, Bool)
                    send.send(msg_sended)
                    virtual_msg = Bool()
                    virtual_msg.data = False
                    self._virtual_send.send(virtual_msg)

class RobotDetect0(RobotDetect):
    def __init__(self):
        RobotDetect.__init__(self, node_name = 'robot_0_detect', receive_topic = 'robot_0/image', send_topic = 'robot_0/follow_sensory_feedback', virtual_off='robot_0/patrol_sensory_feedback')
        
    def run(self):
        RobotDetect.start_detect(self)
        
class RobotDetect1(RobotDetect):
    def __init__(self):
        RobotDetect.__init__(self, node_name = 'robot_1_detect', receive_topic = 'robot_1/image', send_topic = 'robot_1/follow_sensory_feedback', virtual_off='robot_1/patrol_sensory_feedback')
        
    def run(self):
        RobotDetect.start_detect(self)

class RobotDetect2(RobotDetect):
    def __init__(self):
        RobotDetect.__init__(self, node_name = 'robot_2_detect', receive_topic = 'robot_2/image', send_topic = 'robot_2/follow_sensory_feedback', virtual_off='robot_2/patrol_sensory_feedback')
        
    def run(self):
        RobotDetect.start_detect(self)
if __name__ == '__main__':
    robot_detect_0_ob = RobotDetect0()
    robot_detect_0_ob.start()

        


