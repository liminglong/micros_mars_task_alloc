#!/usr/bin/env python
# coding=utf-8

import multiprocessing
import time
import rospy

class Process1(multiprocessing.Process):
    def __init__(self):
        multiprocessing.Process.__init__(self)
        rospy.init_node('hello1')


    def run(self):        
        #rospy.init_node('hello1')
        print 'hello1'
        rospy.spin()

class Process2(multiprocessing.Process):
    def __init__(self):
        multiprocessing.Process.__init__(self)

    def run(self):
        rospy.init_node('hello2')
        print 'hello2'
        rospy.spin() 

if __name__ == '__main__':
    p1 = Process1()
    p2 = Process2()
    p1.start()
    p2.start()
