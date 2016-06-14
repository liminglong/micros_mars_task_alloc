#!/usr/bin/env python
# coding=utf-8
#!/usr/bin/env python
# coding=utf-8


'''
import sys
sys.path.append("..")
from middle_abstraction.function_unit import FunctionUnit
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class RobotDetect(FunctionUnit):
    def __init__(self, node_name, receive_topic, send_topic):
        FunctionUnit.__init__(self, node_name)    
        self._receive_topic = receive_topic
        self._send_topic = send_topic
        self.br = CvBridge()
        
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
                    print 'Detect an intruder!'      
                    msg_sended = Bool()
                    msg_sended.data = True             
                    send = FunctionUnit.create_send(self, self._send_topic, Bool)
                    send.send(msg_sended)

class RobotDetect0(RobotDetect):
    def __init__(self):
        RobotDetect.__init__(self, node_name = 'robot_1_detect', receive_topic = 'robot_0/image', send_topic = 'robot_0/detected')
        
    def run(self):
        RobotDetect.start_detect(self)
        
    
if __name__ == '__main__':
    robot_detect_0_ob = RobotDetect0()
    robot_detect_0_ob.start()
'''

__author__ = 'Minglong Li'

import sys
sys.path.append("..")

from middle_abstraction.function_unit import FunctionUnit
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotFollow(FunctionUnit):
    def __init__(self, node_name, receive_topic_follower, receive_topic_intruder, send_topic):
        FunctionUnit.__init__(self, node_name)    
        self._receive_topic_follower = receive_topic_follower
        self._receive_topic_intruder = receive_topic_intruder
        self.follower_pose_x = 0.0
        self.follower_pose_y = 0.0
        self.intruder_pose_x = 0.0
        self.intruder_pose_y = 0.0 
        self.follow_distance = 0.5 # The follower will be away from the intruder according to this distance      
        self.linear_velocity = 0.3
        self.angular_velocity = 0.5
        self.send = FunctionUnit.create_send(self, send_topic, Twist)
        #self.follower = 'robot_0'
        #self.intruder = 'robot_3'

    def run(self):
        pass

    def start_follow(self):
        FunctionUnit.init_node(self)
        #print 'hello 1'
        #print self._receive_topic
        receive_1 = FunctionUnit.create_receive(self, self._receive_topic_follower, Odometry, self.receive_1_cb)
        receive_2 = FunctionUnit.create_receive(self, self._receive_topic_intruder, Odometry, self.receive_2_cb)
        #print 'hello 2'
        FunctionUnit.spin(self)    

    def receive_1_cb(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w 
        #print "self.intruder_pose_y, self.follower_pose_y, self.intruder_pose_x, self.follower_pose_x: %f %f %f %f" %(self.intruder_pose_y, self.follower_pose_y, self.intruder_pose_x, self.follower_pose_x)
        #print "2*(w*z+x*y), 1-2*(y**2+z**2): %f %f" %( 2*(w*z+x*y), 1-2*(y**2+z**2) )
        #print "self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x: %f %f" %(self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x)
        leader_direction = math.atan2(self.intruder_pose_y - self.follower_pose_y, self.intruder_pose_x - self.follower_pose_x)
        if (leader_direction > -1*math.pi or leader_direction == -1*math.pi) and leader_direction < 0:
            leader_direction = 2*math.pi + leader_direction

        follower_orientation = math.atan2(2*(w*z+x*y), 1-2*(y**2+z**2))
        if (follower_orientation > -1*math.pi or follower_orientation  == -1*math.pi) and follower_orientation < 0:
            follower_orientation = 2*math.pi + follower_orientation
        #print 'follower_orientation: %f' %(follower_orientation)
        #print 'leader_direction: %f' %(leader_direction)

        self.follower_pose_x = msg.pose.pose.position.x
        self.follower_pose_y = msg.pose.pose.position.y
        if ((self.follower_pose_x - self.intruder_pose_x)**2 + (self.follower_pose_y - self.intruder_pose_y)**2) > self.follow_distance ** 2:
            if follower_orientation < leader_direction:  
                cmd = Twist()
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = self.angular_velocity
                self.send.send(cmd)
            else:
                cmd = Twist()
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = -1 * self.angular_velocity
                self.send.send(cmd)
        else:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.send.send(cmd)
                        
    def receive_2_cb(self, msg):
        self.intruder_pose_x = msg.pose.pose.position.x
        self.intruder_pose_y = msg.pose.pose.position.y

class RobotFollow0(RobotFollow):
    def __init__(self):
        RobotFollow.__init__(self, node_name='robot_follow_0', receive_topic_follower='robot_0/base_pose_ground_truth', 
                             receive_topic_intruder='robot_3/base_pose_ground_truth', send_topic='robot_0/cmd_vel/follow')

    def run(self):
        RobotFollow.start_follow(self)

class RobotFollow1(RobotFollow):
    def __init__(self):
        RobotFollow.__init__(self, node_name='robot_follow_1', receive_topic_follower='robot_1/base_pose_ground_truth', 
                             receive_topic_intruder='robot_3/base_pose_ground_truth', send_topic='robot_1/cmd_vel/follow')

    def run(self):
        RobotFollow.start_follow(self)

class RobotFollow2(RobotFollow):
    def __init__(self):
        RobotFollow.__init__(self, node_name='robot_follow_2', receive_topic_follower='robot_2/base_pose_ground_truth', 
                             receive_topic_intruder='robot_3/base_pose_ground_truth', send_topic='robot_2/cmd_vel/follow')

    def run(self):
        RobotFollow.start_follow(self)

if __name__ == '__main__':
    robot_follow_0_ob = RobotFollow0()
    robot_follow_0_ob.start()

