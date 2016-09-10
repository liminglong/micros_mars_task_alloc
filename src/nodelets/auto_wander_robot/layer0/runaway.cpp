/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 11th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>

#include <micros_mars_task_alloc/Force.h>
#include <micros_mars_task_alloc/Heading.h>
#include <pluginlib/class_list_macros.h>

namespace micros_mars_task_alloc {
using namespace std;

class Runaway : public nodelet::Nodelet
{
public:
    Runaway(): significant_force_(0.5){}
    ~Runaway(){}
    virtual void onInit();
    void callback(const micros_mars_task_alloc::ForcePtr & msg);
private:
    float significant_force_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

void Runaway::onInit()
{
    nh_ = getPrivateNodeHandle();
    cout << "Initialising nodelet ..." << endl;        
    sub_ = nh_.subscribe("/feelforce/force", 10, &Runaway::callback, this);
    pub_ = nh_.advertise<micros_mars_task_alloc::Heading>("test/turn/heading", 10);//The topic prefix runaway (nodelet name) will be added automically.
}

void Runaway::callback(const micros_mars_task_alloc::ForcePtr & msg)
{
    micros_mars_task_alloc::HeadingPtr  heading_ptr(new micros_mars_task_alloc::Heading);
    heading_ptr -> distance = msg -> magnitude;
    heading_ptr -> angle = msg -> direction;
    pub_.publish(heading_ptr);
    //cout << "hello" << endl;
}

}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Runaway, nodelet::Nodelet)
