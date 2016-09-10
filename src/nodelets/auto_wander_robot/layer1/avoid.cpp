/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 18th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>

#include <math.h>

#include <micros_mars_task_alloc/Heading.h>
#include <micros_mars_task_alloc/Force.h>

namespace micros_mars_task_alloc{
using namespace std;

class Avoid : public nodelet::Nodelet
{
public:
    Avoid(): magnitude_(0.0), direction_(0.0), random_factor_(0.8), pi_(std::acos(-1)){}
    ~Avoid(){}
    
    virtual void onInit();
    
    void callback_0(const micros_mars_task_alloc::HeadingConstPtr& msg);
    void callback_1(const micros_mars_task_alloc::ForceConstPtr& msg);
     

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_0_;
    ros::Subscriber sub_1_;    
    
    float magnitude_;//magnitude of force_msg;
    float direction_;//direction of force_msg;
    float pi_;
    float random_factor_;//proportion of the heading message.
};

void Avoid::onInit()
{
    nh_ = getPrivateNodeHandle();
    sub_0_ = nh_.subscribe("/wander/heading", 10, &Avoid::callback_0, this);
    sub_1_ = nh_.subscribe("/feelforce/force", 10, &Avoid::callback_1, this);
    pub_ = nh_.advertise<micros_mars_task_alloc::Heading>("suppressor/heading", 10);//the topic here is a relative topic, and the prefix "avoid" will be added.
    
}

void Avoid::callback_0(const micros_mars_task_alloc::HeadingConstPtr& msg)
{
    micros_mars_task_alloc::HeadingPtr avoid_heading_ptr(new micros_mars_task_alloc::Heading);
    
    float x0, y0, x1, y1, x_average, y_average;
    x0 = 0.0;
    y0 = 0.0;
    x1 = 0.0;
    y1 = 0.0;
    x_average = 0.0;
    y_average = 0.0;
    
    x0 = float(double((msg -> distance)) * std::cos(double(msg -> angle)));
    y0 = float(double((msg -> distance)) * std::sin(double(msg -> angle)));
    
    x1 = float(double(magnitude_) * std::cos(double(direction_)));
    y1 = float(double(magnitude_) * std::sin(double(direction_)));
    
    x_average = random_factor_ * x0 + (1 - random_factor_) * x1;
    y_average = random_factor_ * y0 + (1 - random_factor_) * y1;

    avoid_heading_ptr -> distance = float(std::sqrt(double(x_average * x_average + y_average * y_average)));
    avoid_heading_ptr -> angle = float(std::fmod( (std::atan2(double(y_average), double(x_average)) + double(2 * pi_)), double(2 * pi_) ));
    
    pub_.publish(avoid_heading_ptr);
    cout << "The avoid heading message was published successfully!" << endl;
}

void Avoid::callback_1(const micros_mars_task_alloc::ForceConstPtr& msg)
{
    magnitude_ = msg -> magnitude;
    direction_ = msg -> direction;
}

}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Avoid, nodelet::Nodelet)
