#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import rospy
import node
import send
import receive
from std_msgs.msg import String

class FunctionUnit(node.Node):
    def __init__(self, node_name):
        #node.__init__(self)
        super(FunctionUnit, self).__init__()
        self._node_name = node_name

    def run(self):
        rospy.init_node(self._node_name)
        send1 = send.Send('hello_world', String)
        receive1 = receive.Receive('hello_world_2', String)#TODO:test hello

#只有继承才能重写父类的方法，关联关系不是父类，也不能重写里面的方法。
'''    
    def receive1.on_received(msg):
        print "On_received method start!"
        print msg.data
'''
