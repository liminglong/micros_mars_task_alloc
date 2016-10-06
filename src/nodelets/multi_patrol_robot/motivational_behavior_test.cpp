/*
Authot: Minglong Li
Date: Oct. 6th, 2016
*/
#include "motivational_behavior.h"
#include <std_msgs/Bool.h>

namespace micros_mars_task_alloc {
typedef micros_mars_task_alloc::MotivationalBehavior<std_msgs::Bool> MotivationalBehaviorTest;
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::MotivationalBehaviorTest, nodelet::Nodelet)