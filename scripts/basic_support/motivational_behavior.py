#!/usr/bin/env python
# coding = utf-8

__author__ = "Minglong Li"

import sys
sys.path.append("..")
from middle_abstraction.function_unit import FunctionUnit
from micros_mars_task_alloc.msg import Heartbeat
from std_msgs.msg import Bool
import time
import thread
import rospy
import random

class MotivationalBehavior(FunctionUnit):
    active_behavior = [0 for row in range(10)]
    def __init__(self, node_name, robot_id, behavior_id, switch_topic, heartbeat_topic = None):
        if heartbeat_topic == None:
            self._heartbeat_topic = 'heart_beat'
         #param
        self._fast = 20#fast rate, constant for now
        self._slow = 3#slow rate,constant for now
        self._comm_influence_time=10
        self._comm_keep_time=20
        self._reset_influence_time=10
        self._acquienscence_time_0=20#if other robot is runing the same behavior
        self._acquienscence_time_1=60
        self._acquienscence_influence_time=5
        self._robot_id=robot_id#i
        self._behavior_id=behavior_id#j
        self._switch_topic=switch_topic
        self._broadcast_rate=5
        self._random_inc = False

        self._node_name=node_name
        FunctionUnit.__init__(self, node_name)
        self._threshold_of_activation = 100#The initial value of threshold_of_activation is 100.
        self._sensory_feedback = 0#If the sensory information is valid, the sensory feedback is true, the value is 1 or 0.
        self._comm_recieved = [[0 for col in range(1)] for row in range(10)]#_comm_received,row=k,col=t
        self._activity_suppression = 1        
        self._impatience = self._fast
        self._impatience_reset = 1#if this robot is running an other behavior,reset=0
        self._acquiescence = 1#if this behavior has run too long, acquiescence=0
        self._m = 0#final score
        self._time = 0
        self._active=0#bool, active=0 else=1
        self._active_time = 0#when this behavior is active
       
        #self.
        #self.sensory_feedback = 'topic...'

    def run(self):
        FunctionUnit.init_node(self)
        thread.start_new_thread(self.timer,())
        heartbeat_receive = FunctionUnit.create_receive(self, self._heartbeat_topic, Heartbeat, self.heartbeat_on_received)
        self._heartbeat_send = FunctionUnit.create_send(self, 'heart_beat', Heartbeat)
        self._switch_send = FunctionUnit.create_send(self, self._switch_topic, Bool)
        #sensory_receive = FunctionUnit.create_receive(self, self._node_name+"/sensory_feedback", Bool, self.sensory_on_received)
        FunctionUnit.spin(self)

   # def start_motive(self):
        #FunctionUnit.init_node(self)
        #heartbeat_receive = FunctionUnit.create_receive(self, self._heartbeat_topic, Heartbeat, self.heartbeat_on_received)
        #FunctionUnit.spin(self)

    def heartbeat_on_received(self,msg):
        if msg.behavior_set_ID == self._behavior_id and msg.heartbeat == True:
              self._comm_recieved[msg.robot_ID][self._time] = 1
              print "hb msg rcv"
        if msg.robot_ID == self._robot_id:
              if msg.heartbeat == True:
                   MotivationalBehavior.active_behavior[msg.behavior_set_ID] = 1
              else:
                   MotivationalBehavior.active_behavior[msg.behavior_set_ID] = 0

    def sensory_on_received(self,msg):
        #print "sensory msg rcv"
        if msg.data:
             self._sensory_feedback = 1
        else:
             self._sensory_feedback = 0

    def timer(self):
        while not rospy.is_shutdown():
            time.sleep(1)
            for i in range(0,10):
                self._comm_recieved[i].append(0)
            self._time = self._time + 1
            #self._sensory_feedback = 1#constant for now
            self.calculate_activity_suppression()
            #self._fast = 10#constant for now
            #self._slow = 3#constant for now
            if self._random_inc:
                self._fast = random.randint(10,40)
            self.calculate_impatience()
            self.calculate_impatience_reset()
            self.calculate_acquiescence()
            self._m = (self._m + self._impatience) * self._sensory_feedback * self._activity_suppression * self._impatience_reset * self._acquiescence
            if self._m > self._threshold_of_activation:
                if self._active==0:
                   print "activate switch,send hb msg"
                   self.send_heartbeat(True)
                   self._switch_send.send(True)
                   self._active_time=self._time
                self._active=1
            else:
                if self._active==1:
                   self._active=0
                   self.send_heartbeat(False)
                   self._switch_send.send(False)
                   print "turn off switch,send hb msg "
            if self._time % self._broadcast_rate == 1:
                if self._active == 1:
                   print "active, send hb msg"
                   self.send_heartbeat(True)
            print self._time
            print "set%d time%d: %d sup%d impare%d acq%d inc%d sensor%d actime%d"%(self._behavior_id, self._time, self._m, self._activity_suppression, self._impatience_reset, self._acquiescence, self._impatience, self._sensory_feedback, self._active_time)
   
    def calculate_activity_suppression(self):
         mark=0
         for i in range(0,10):
             if i != self._behavior_id and MotivationalBehavior.active_behavior[i] == 1:
                  mark=1
         if mark == 0:
             self._activity_suppression = 1
         else:
             self._activity_suppression = 0
     
    def calculate_impatience(self):
          mark=0
          start_time=self._time - self._comm_influence_time
          end_time=self._time - self._comm_keep_time
          if start_time < 0:
              start_time = 0
          if end_time < 0:
              end_time = 0
          for i in range(0,10):
              if i != self._robot_id:
                   mark1=0
                   for j in range(start_time, self._time+1):
                        if self._comm_recieved[i][j] == 1:
                             mark1=1
                   #mark2=0
                   #for k in range(0,end_time):
                   #     if self._comm_recieved[i][k] ==1:
                   #          mark2=1
                   if mark1 == 1: #and mark2 == 0:
                        mark=1#assume _slow is constant for diferrent robots
          if mark == 1:
              self._impatience = self._slow
          else:
              self._impatience = self._fast
         
    def calculate_impatience_reset(self):
          mark=0
          start_time=self._time - self._reset_influence_time
          if start_time < 0:
              start_time = 0
          for i in range(0,10):
              if i != self._robot_id:
                   mark1=0
                   for j in range(start_time, self._time+1):
                        if self._comm_recieved[i][j] == 1:
                             mark1=1
                   mark2=0
                   for k in range(0,start_time):
                        if self._comm_recieved[i][k] ==1:
                             mark2=1
                   if mark1 == 1 and mark2 == 0:
                        mark=1
          if mark ==1:
              self._impatience_reset = 0
          else:
              self._impatience_reset = 1

    def calculate_acquiescence(self):
          if self._active == 0:
              self._acquiescence = 1#not active
          else:
              if self._time - self._active_time > self._acquienscence_time_1:#time>time1
                  self._acquiescence = 0
              else:
                  if self._time - self._active_time > self._acquienscence_time_0:#time>time0,check
                      mark=0
                      start_time=self._time - self._acquienscence_influence_time
                      if start_time < 0:
                          start_time = 0
                      for i in range(0,10):
                          if i != self._robot_id:
                              for x in range(start_time, self._time+1):
                                  if self._comm_recieved[i][x] ==1:
                                      mark = 1
                      if mark == 1:
                         self._acquiescence = 0
                      else:
                         self._acquiescence = 1
                  else:
                      self._acquiescence = 1
    def send_heartbeat(self,hb):
          msg=Heartbeat()
          msg.robot_ID=self._robot_id
          msg.behavior_set_ID=self._behavior_id
          msg.heartbeat=hb
          self._heartbeat_send.send(msg)

    def set_fast(self,fast):
          self._fast=fast

    def set_sensory_feedback(self,value):
          self._sensory_feedback = value

    def set_sensor(self,sensor_topic):
          sensory_receive = FunctionUnit.create_receive(self, sensor_topic, Bool, self.sensory_on_received)

    def enable_random_inc(self):
          self._random_inc=True
