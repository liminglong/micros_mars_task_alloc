/*************************************************************************
	> File Name: whenlook.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
	> Created Time: July 18th, 2016
 ************************************************************************/

#include<iostream>
#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<std_msgs/Bool.h>

namespace micros_mars_task_alloc{
    class Whenlook : public nodelet::Nodelet{
    public:
        virtual void onInit();
        void callback(const std_msgs::BoolConstPtr & msg);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
    };
    
    void Whenlook::onInit()
    {
        nh_ = getPrivateNodeHandle();
        sub_ = nh_.subscribe("/status/busy", 10, &Whenlook::callback, this);//this topic is an absolute topic
        pub_ = nh_.advertise<std_msgs::Bool>("startlook", 10);//this topic is a relative topic, the prefix whenlook will be added.
    }
    void Whenlook::callback(const std_msgs::BoolConstPtr & msg)
    {
        if (msg->data == false)
        {
            std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
            bool_ptr -> data = true;
            pub_.publish(bool_ptr);
        }
    }
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Whenlook, nodelet::Nodelet)
