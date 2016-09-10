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
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <micros_mars_task_alloc/Heading.h>

namespace micros_mars_task_alloc {
using namespace std;

class Forward : public nodelet::Nodelet
{
public:
    Forward(): halt_flag_(false), angular_velocity_(1.0), linear_velocity_(0.5), idle_time_(0.5){}
    ~Forward(){}

    virtual void onInit();
    void forward_CB(const micros_mars_task_alloc::HeadingConstPtr& msg);
    void halt_CB(const std_msgs::BoolConstPtr& msg);
    
private:
    bool halt_flag_;
    float linear_velocity_;
    float angular_velocity_;
    double idle_time_;

    //float pi_;
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_0_;
    ros::Subscriber sub_1_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_turn_reset_;
    ros::Publisher pub_busy_;
   
};

void Forward::onInit()
{
    nh_ = getPrivateNodeHandle();
    cout << "Initialising nodelet ..." << endl;
    sub_0_ = nh_.subscribe("/turn/heading", 10, &Forward::forward_CB, this);
    sub_1_ = nh_.subscribe("/collide/halt", 10, &Forward::halt_CB, this);
    
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 10);//the topic here is a absolute topic.
    pub_turn_reset_ = nh_.advertise<std_msgs::Bool>("/turn/reset", 10);//the topic here is a relative topic, the prefix "turn" will be added.
    pub_busy_ = nh_.advertise<std_msgs::Bool>("/robot/busy", 10);// the topic here is a absolute topic.
}

void Forward::forward_CB(const micros_mars_task_alloc::HeadingConstPtr& msg)
{
    cout << "forward_CB start!" << endl;
    cout << "halt_flag_: " << halt_flag_ << endl;
    ros::Rate rate(10);//10Hz
    geometry_msgs::TwistPtr twist_ptr(new geometry_msgs::Twist);
    std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
    std_msgs::BoolPtr busy_ptr(new std_msgs::Bool);
    
    double forward_count = 0;
    cout << "while() start" << endl;
    float distance  = msg->distance;
    if(isnan(msg-> distance))
    {
        distance = 0.5;
    } 
    while(ros::ok())
    {
        twist_ptr -> linear.x = linear_velocity_;
        pub_twist_.publish(twist_ptr);
        forward_count += 1;
        cout << "msg->angle in forward module: " << msg->angle << endl;
        cout << "msg->distance in forward moudle: " << msg->distance << endl;
        cout << "linear_velocity_ * 0.1 * forward_count in forwarde module: " << linear_velocity_ * 0.1 * forward_count << endl;
        if (halt_flag_)
        {
            twist_ptr -> linear.x = 0.0;
            pub_twist_.publish(twist_ptr);
            bool_ptr -> data = true;
            pub_turn_reset_.publish(bool_ptr);
            cout << "turn_reset 'true' was published successfully." << endl;
            break;
        }
        else if(linear_velocity_ * 0.1 * forward_count > distance)
        {
            cout << " 'else if' module start!" << endl;
            forward_count = 0;
            twist_ptr->linear.x = 0.0;
            pub_twist_.publish(twist_ptr);
            cout << "robot stop, the robot is idle" << endl;
            //the robot status is idle here.
            busy_ptr ->data = false;
            pub_busy_.publish(busy_ptr);
            ros::Duration(idle_time_).sleep();
            //pub return message to the Turn module.
            bool_ptr -> data = true;
            pub_turn_reset_.publish(bool_ptr);
            cout << "turn reset was published successfully in forwardCB";
            break;
        }//如果算出来的距离过小，使其<distance，那么这个模块不执行，reset_flag被置为false，turn模块也不执行，就停住了。这是对的，找一块儿空旷地带，停在那里了。
        rate.sleep();
    }
    cout << "while() end" << endl;
}

void Forward::halt_CB(const std_msgs::BoolConstPtr& msg)
{
    halt_flag_ = msg -> data;
    if(halt_flag_)
    {
        std_msgs::BoolPtr bool_ptr(new std_msgs::Bool); 
        //geometry_msgs::TwistPtr twist_ptr(new geometry_msgs::Twist);   
        bool_ptr -> data = true;
        pub_turn_reset_.publish(bool_ptr);
        cout << "turn reset message was published successfully in halt_CB"<< endl;
    }
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Forward, nodelet::Nodelet)
