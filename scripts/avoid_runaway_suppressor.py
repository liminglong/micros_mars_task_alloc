#!/usr/bin/env python
#coding=utf-8
import roslib
import rospy
import time

from subsumption_msgs.msg import Heading 
from subsumption_msgs.msg import Force

time_unit = 0.1
time_unit_amount = 20
time_start = 0.0

def avoid_headingCB(msg):
	global time_start
	time_start = time.time()
	pub_heading = rospy.Publisher('/suppressor/turn/heading', Heading, queue_size=10)	
	pub_heading.publish(msg)	
	print "The upper layer published successfully!"	

def runaway_headingCB(msg):
	global time_unit
	global time_unit_amount
	global time_start
	time_end = time.time()
	pub_heading = rospy.Publisher('/suppressor/turn/heading', Heading, queue_size=10)	
	if (time_end - time_start) > (time_unit * time_unit_amount):
		pub_heading.publish(msg)
		print "The upper layer didn't work, the lower layer instead!"

def avoid_runaway_suppressor():
	rospy.init_node('avoid_runaway_suppressor')
	rospy.Subscriber("/avoid/suppressor/heading", Heading, avoid_headingCB)	
	rospy.Subscriber("/runaway/suppressor/heading", Heading, runaway_headingCB)
	rospy.spin()

if __name__ == '__main__':
	try:
		avoid_runaway_suppressor()
	except rospy.ROSInterruptException:
		print "Program interrupted before completion"


