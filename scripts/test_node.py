#!/usr/bin/env python
# coding=utf-8

__author__ = 'Minglong Li'

import node
from std_msgs import String

#把node类拆开


class Test_node(Node):
    def __init__(self):
        self.on_received_1 = On_received('hello', String)
        spin()

    #def self.on_received_1.action(self, msg):
        #print msg.data


if __name__ == '__main__':
	try:
		test_node = Node('hello')
        on_received_1 = On_received()
        on_received_2 = On_received()
        test_node.start()
	except rospy.ROSInterruptException:
		print "Program interrupted before completion"

