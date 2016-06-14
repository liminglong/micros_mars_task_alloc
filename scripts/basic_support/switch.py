#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import sys
sys.path.append("..")
from middle_abstraction.function_unit import FunctionUnit
from std_msgs.msg import Bool
from multi_robot_patrol.msg import Heartbeat
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal

class Switch(FunctionUnit):
    def __init__(self, node_name, topic_1 = None, msg_type_1 = None, send_topic_1 = None, topic_2 = None, msg_type_2 = None, send_topic_2 = None):
        
        self._motive_topic_1=node_name+'/activate'
        if not(topic_1 == None) and not(msg_type_1 == None):
            self._topic_1 = topic_1
            self._msg_type_1 = msg_type_1
            self._send_topic_1=send_topic_1
            self.send_1 = FunctionUnit.create_send(self, self._send_topic_1, self._msg_type_1)
        else:
            self._topic_1 = None
            self._msg_type_1 = None            

        if not(topic_2 == None) and not(msg_type_2 == None):
            self._topic_2 = topic_2
            self._msg_type_2 = msg_type_2
            self._send_topic_2=send_topic_2
            self.send_2 = FunctionUnit.create_send(self, self._send_topic_2, self._msg_type_2)
        else:
            self._topic_2 = None
        
        #print 'init'
        FunctionUnit.__init__(self, node_name)
        self.motivational_shared_var = False #'True' means the behavior set is activated
        self._action_mode = False
        self._last_goal = GoalID()

    def run(self):
        self.start_switch()
        pass

    def start_switch(self):
        FunctionUnit.init_node(self)
        #print 'run'
        if not(self._topic_1 == None):
            receive_1 = FunctionUnit.create_receive(self, self._topic_1, self._msg_type_1, self.receive_cb_1)
        if not(self._topic_2 == None):
            receive_2 = FunctionUnit.create_receive(self, self._topic_2, self._msg_type_2, self.receive_cb_2)
        receive_3 = FunctionUnit.create_receive(self, self._motive_topic_1, Bool, self.motivational_shared_var_update)
        FunctionUnit.spin(self)

    def motivational_shared_var_update(self, msg):
        self.motivational_shared_var = msg.data
        if msg.data==False:
           print "switch off"
           if self._action_mode:
              self._send_cancel.send(self._last_goal)
              print "cancel_goal"
        else:
           print "switch on"

    def receive_cb_1(self, msg):
        if self.motivational_shared_var == True:
            self.send_1.send(msg)

    def receive_cb_2(self, msg):#这里有可能会报错，因为receive_cb_2有可能是空的，根本不知道传到哪里，不过应该也没关系。
        if self.motivational_shared_var == True:    
            self.send_2.send(msg)

    def add_action(self,sub_action_name,pub_action_name):
        #self._action=action_client
        self._action_mode=True
        self._recieve_action_cancel = FunctionUnit.create_receive(self, sub_action_name + '/goal', MoveBaseActionGoal, self.receive_cb_goal)
        self._send_cancel= FunctionUnit.create_send(self, pub_action_name+'/cancel', GoalID)

    def receive_cb_goal(self,msg):
        print 'goal rcv'
        #if self.motivational_shared_var == True:    
        self._last_goal.stamp.secs = msg.goal_id.stamp.secs
        self._last_goal.stamp.nsecs = msg.goal_id.stamp.nsecs
        self._last_goal.id = msg.goal_id.id

