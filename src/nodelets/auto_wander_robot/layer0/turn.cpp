/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 16th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <micros_mars_task_alloc/Heading.h>

namespace micros_mars_task_alloc {
using namespace std;

class Turn : public nodelet::Nodelet
{
public:
    Turn(): pi_(std::acos(-1)), reset_flag_(true), angular_velocity_(1.0), turn_frequency_(50){}
    ~Turn(){}

    virtual void onInit();
    void turn_CB(const micros_mars_task_alloc::HeadingConstPtr& msg);
    void reset_CB(const std_msgs::BoolConstPtr& msg);
    
private:
    float pi_;
    bool reset_flag_;
    float angular_velocity_;
    float turn_frequency_;
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_0_;
    ros::Subscriber sub_1_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_heading_;
    
    
    
};

void Turn::onInit()
{
    nh_ = getPrivateNodeHandle();
    cout << "Initialising nodelet ..." << endl;
    sub_0_ = nh_.subscribe("/runaway/turn/heading", 10, &Turn::turn_CB, this);//the topic is /turn/runaway/heading
    sub_1_ = nh_.subscribe("/turn/reset", 10, &Turn::reset_CB, this);
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 10);//the topic here is a absolute topic
    pub_heading_ = nh_.advertise<micros_mars_task_alloc::Heading>("/turn/heading", 10);//the topic here is a relative topic
}

void Turn::turn_CB(const micros_mars_task_alloc::HeadingConstPtr& msg)
{
    float turn_angle = msg -> angle;
    float count;
    ros::Rate loop_rate(turn_frequency_);
    cout << "msg->angle in turn module: " << msg->angle << endl;
    cout << "msg->distance in turn module: " << msg->distance << endl;
    cout << "reset_flat_: " << reset_flag_ << endl;
    if(isnan(turn_angle))
    {
        turn_angle = 1.57;//pi/2
    } 
    if(reset_flag_)
    {
        cout << "turn reset start" << endl;
        geometry_msgs::TwistPtr twist_ptr(new geometry_msgs::Twist);
        if (turn_angle == 0.0 || (turn_angle > 0.0 && turn_angle < pi_) || turn_angle == pi_)
        {
            cout << "turn left start" << endl;
            twist_ptr -> angular.z = angular_velocity_;//TODO: angular.z right???????????????????
            count = 0;
            cout << "turn angle is: " << turn_angle << endl;
            while(ros::ok())
            {
                pub_twist_.publish(twist_ptr);
                count += 1;
                loop_rate.sleep();
                if ((1.0 / turn_frequency_) * count * angular_velocity_ > turn_angle)
                {
                    twist_ptr -> angular.z = 0;
                    pub_twist_.publish(twist_ptr);
                    break;
                }
            }
            count = 0;
        }
        else
        {
            cout << "turn right start" << endl; 
            twist_ptr -> angular.z = (-1.0) * angular_velocity_;
            count = 0;
            cout << "turn angle is: " << turn_angle << endl;
            cout << "(1.0 / turn_frequency_) * count * angular_velocity_　: " << (1.0 / turn_frequency_) * count * angular_velocity_ << endl;
            cout << "(2 * pi_ - turn_angle): " << (2 * pi_ - turn_angle) << endl;
            while(ros::ok())
            {
                pub_twist_.publish(twist_ptr);
                count += 1;
                loop_rate.sleep();
                if ((1.0 / turn_frequency_) * count * angular_velocity_ > (2 * pi_ - turn_angle))
                    twist_ptr -> angular.z = 0;
                    pub_twist_.publish(twist_ptr);
                    break;
            }
            count = 0;
        }
        cout << "turn reset stop" << endl;
        float temp_distance = msg->distance;
        micros_mars_task_alloc::HeadingPtr heading_ptr2(new micros_mars_task_alloc::Heading);
        heading_ptr2->distance = temp_distance;
        pub_heading_.publish(heading_ptr2);
        cout << "heading message was published successfully to the forward module." << endl;
        reset_flag_ = false;
    }
}

void Turn::reset_CB(const std_msgs::BoolConstPtr& msg)
{
    reset_flag_ = msg -> data;
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Turn, nodelet::Nodelet)
