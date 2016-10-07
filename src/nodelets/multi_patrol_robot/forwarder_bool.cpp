/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: Sep. 5th, 2016
*/
#include "forwarder.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Bool.h>

namespace micros_mars_task_alloc {

typedef micros_mars_task_alloc::Forwarder<std_msgs::Bool> ForwarderTest;

}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::ForwarderTest, nodelet::Nodelet)
