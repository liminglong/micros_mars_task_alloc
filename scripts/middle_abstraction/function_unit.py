#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import rospy
import node
import send
import receive
from std_msgs.msg import String

class FunctionUnit(node.Node, receive.Receive, send.Send):
    def __init__(self, node_name):
        #node.__init__(self)
        super(FunctionUnit, self).__init__()
        self._node_name = node_name

    def init_node(self):
        rospy.init_node(self._node_name)

    def spin(self):
        rospy.spin()    

    def run(self):
        pass
        #rospy.init_node(self._node_name)
        #send1 = send.Send('hello_world', String)
        #receive1 = receive.Receive('hello_world_2', String, self.on_received1)#把FunctionUnit中的函数名，
        #rospy.spin()
        
    def create_receive(self, topic, msg_type, on_received):
        return receive.Receive(topic, msg_type, on_received)
     
    def create_send(self, topic, msg_type):   
        return send.Send(topic, msg_type)

    def rebuild_receive(self, receive, old_msg_type, new_topic):
        receive.unregister()
        del receive
        return receive.Receive(new_topic, old_msg_type, on_received)#怎么把之前的回调函数传进来？传回调函数对象的时候应该在后面实例化的时候。
    '''
    def on_received1(self, msg):
        print "message received"
        print msg
        #receive1.start()
        #MARK:test hello
    '''
#Only by inheriting can we rewrite the method of parent class
'''    
    def receive1.on_received(msg):
        print "On_received method start!"
        print msg.data
'''
