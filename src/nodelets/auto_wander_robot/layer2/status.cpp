/*************************************************************************
	> File Name: status.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
    > Created on: July 18th, 2016
 ************************************************************************/

#include<iostream>
#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>

#include<std_msgs/Bool.h>

namespace micros_mars_task_alloc{
    using namespace std;
    
    class Status : public nodelet::Nodelet
    {
    public:
        Status(){}
        ~Status(){}

        virtual void onInit();
        void callback(const std_msgs::BoolConstPtr & msg);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
    };

    void Status::onInit()
    {
        nh_ = getPrivateNodeHandle();
        cout << "Initialising nodelet ..." << endl;

        sub_ = nh_.subscribe("/robot/busy", 10, &Status::callback, this);//the topic is a absolute topic.
        pub_ = nh_.advertise<std_msgs::Bool>("busy", 10);//the topic is a relative topic, and the prefix "status" will be added.
    }
    void Status::callback(const std_msgs::BoolConstPtr & msg)
    {
        ros::Rate rate(10);
        int count = 0;
        std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
        bool_ptr -> data = msg -> data;
        pub_.publish(bool_ptr);
        /*
        while (ros::ok())
        {
            std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
            bool_ptr -> data = msg -> data;
            pub_.publish(bool_ptr);
            count += 1;
            rate.sleep();
            if (count == 5)
            {
                count = 0;
                break;
            }
        }
        */
    }
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Status, nodelet::Nodelet)
