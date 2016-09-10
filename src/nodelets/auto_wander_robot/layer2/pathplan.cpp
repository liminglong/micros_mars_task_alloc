/*************************************************************************
	> File Name: pathplan.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
	> Created on: July 18th, 2016
 ************************************************************************/

#include<iostream>
#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<std_msgs/Bool.h>
#include<micros_mars_task_alloc/Heading.h>
#include<micros_mars_task_alloc/Path.h>
#include<pluginlib/class_list_macros.h>

namespace micros_mars_task_alloc{
    using namespace std;
    class Pathplan : public nodelet::Nodelet
    {
    public:
        Pathplan() : start_pathplan_(false){}
        ~Pathplan(){}

        virtual void onInit();
        void callback_0(const micros_mars_task_alloc::PathConstPtr & msg);
        void callback_1(const std_msgs::BoolConstPtr & msg);

    private:    
        ros::NodeHandle nh_;
        ros::Subscriber sub_0_; 
        ros::Subscriber sub_1_;
        ros::Publisher pub_;
        bool start_pathplan_;
    };

    void Pathplan::onInit()
    {
        nh_ = getPrivateNodeHandle();
        sub_0_ = nh_.subscribe("/look/path", 10, &Pathplan::callback_0, this);//This is an absolute topic.
        sub_1_ = nh_.subscribe("/whenlook/startlook", 10, &Pathplan::callback_1, this);//This is an absolute topic.
        pub_ = nh_.advertise<micros_mars_task_alloc::Heading>("suppressor", 10);//This is a relative topic, the prefix pathplan will be added automaically.
    }

    void Pathplan::callback_0(const micros_mars_task_alloc::PathConstPtr & msg)
    {
        if (start_pathplan_)
        {
            micros_mars_task_alloc::HeadingPtr heading_ptr(new micros_mars_task_alloc::Heading);
            heading_ptr -> angle = msg -> path_angle;
            heading_ptr -> distance = msg -> path_distance;
            pub_.publish(heading_ptr);
        }
        start_pathplan_ = false;
    }

    void Pathplan::callback_1(const std_msgs::BoolConstPtr & msg)
    {
        start_pathplan_ = msg -> data;
        /*
        if(start_pathplan_)
        {
            ros::Duration(0.5).sleep();
            start_pathplan_ = false;
        }
        */
        
    }
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Pathplan, nodelet::Nodelet)
