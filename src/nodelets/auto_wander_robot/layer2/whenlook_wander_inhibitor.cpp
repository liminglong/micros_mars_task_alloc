/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 19th, 2016
*/
#include "inhibitor.h"
#include <std_msgs/Bool.h>
#include <micros_mars_task_alloc/Heading.h>

namespace micros_mars_task_alloc {

typedef micros_mars_task_alloc::Inhibitor<std_msgs::Bool, micros_mars_task_alloc::Heading> WhenlookWanderInhibitor;

}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::WhenlookWanderInhibitor, nodelet::Nodelet)
