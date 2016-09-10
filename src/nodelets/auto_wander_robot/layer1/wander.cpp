/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 17th, 2016
*/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <pluginlib/class_list_macros.h>

#include <micros_mars_task_alloc/Heading.h>

namespace micros_mars_task_alloc {
using namespace std;

class Wander : public nodelet::Nodelet
{
public:
    Wander():forward_distance_(3.0){}
    ~Wander(){}

    virtual void onInit();
    void timerCallback(const ros::TimerEvent&);
        
private:
    float forward_distance_;
    
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher pub_;
};

void Wander::onInit()
{
    nh_ = getPrivateNodeHandle();
    pub_ = nh_.advertise<micros_mars_task_alloc::Heading>("/wander/heading", 10);
    cout << "Initialising nodelet ..." << endl;
    timer_ = nh_.createTimer(ros::Duration(1.0), &Wander::timerCallback, this);
}

void Wander::timerCallback(const ros::TimerEvent&)
{
    micros_mars_task_alloc::HeadingPtr heading_ptr(new micros_mars_task_alloc::Heading);
    heading_ptr->distance = forward_distance_;
    srand(time(NULL));
    heading_ptr->angle = (-1000 + rand()%2000)/1000.0; 
    pub_.publish(heading_ptr);
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Wander, nodelet::Nodelet)
