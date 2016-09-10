/*************************************************************************
	> File Name: pathplan.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
	> Created on: July 18th, 2016
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <limits> 
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <micros_mars_task_alloc/Path.h>
#include <math.h>
#include <vector>

#include <pluginlib/class_list_macros.h>

namespace micros_mars_task_alloc{
    using namespace std;
    class Look : public nodelet::Nodelet
    {
    public:
        Look():max_range_(8.0), start_look_(false), travel_max_distance_(10.0), pi_(std::acos(-1)){}
        ~Look(){}

        virtual void onInit();
        
        void callback_0(const sensor_msgs::LaserScanConstPtr & msg);
        void callback_1(const std_msgs::BoolConstPtr & msg);

    private:    
        ros::NodeHandle nh_;
        ros::Subscriber sub_0_; 
        ros::Subscriber sub_1_;
        ros::Publisher pub_;
        
        float max_range_;
        bool start_look_;
        float travel_max_distance_;
        float pi_;
    };

    void Look::onInit()
    {
        nh_ = getPrivateNodeHandle();
        sub_0_ = nh_.subscribe("/robot0/laser_12", 10, &Look::callback_0, this);//This is an absolute topic.
        sub_1_ = nh_.subscribe("/whenlook/startlook", 10, &Look::callback_1, this);//This is an absolute topic.
        pub_ = nh_.advertise<micros_mars_task_alloc::Path>("path", 10);//This is a relative topic, the prefix look will be added automaically.
    }

    void Look::callback_0(const sensor_msgs::LaserScanConstPtr & msg)
    {
        if(start_look_)
        {
            int i =0;
            float turn_angle = 0.0;
            float forward_magnitude = 0.0;
            float max_distance = 0.0;
            int len = msg->ranges.size();//ranges is a vector, get the lenth of ranges
            float ranges[len];
            for(i=0; i<len; i++)
            {
                ranges[i] = msg->ranges[i];
            }
            for(i=0; i<len; i++)
            {
                if (ranges[i] == std::numeric_limits<float>::infinity())
                    ranges[i] = max_range_;
            }
            //get the max value of ranges
            max_distance = ranges[0];
            for(i = 1; i < len; i++)
            {
                if (ranges[i] > max_distance)
                    max_distance = ranges[i];
            } 
            
            //choose a direction
            int count_a = 0;
            int count_b = 0;
            int count_c = 0;
            int count_d = 0;
            int angle_index = 0;
            
            for(i=0; i<len; i++)
            {
                if(ranges[i] == max_distance)
                {
                    count_a = i;
                    break;
                }
            }
            for(i=0; i<len; i++)
            {
                if((ranges[i] == max_distance) && (ranges[i+1] != max_distance))
                {
                    count_b = i;
                    break;
                }
            }
            for(i=count_b+1; i<len; i++)
            {
                if(ranges[i] == max_distance)
                {
                    count_c = i;
                    break;
                }
            }
            for(i=count_b+1; i<len; i++)
            {
                if((ranges[i] == max_distance) && (ranges[i+1] != max_distance))
                {
                    count_d = i;
                    break;
                }
            }
            
            //calculate the turn angle
            if (abs(count_b - count_a) > abs(count_c - count_d))
            {
                angle_index = (count_a + count_b)/2;
                if((angle_index == 0) || ((angle_index > 0) && (angle_index < 135)) )
                    turn_angle = float((225 + angle_index)) * float((pi_ / 180.0));
                else
                    turn_angle = float(angle_index - 135) * float(pi_ / 180.0);
            }   
            else
            {
                angle_index = (count_c + count_d) / 2;
                if ((angle_index == 0) || ((angle_index > 0) && (angle_index < 135)))
                    turn_angle = float(225 + angle_index) * float(pi_ / 180);
                else
                    turn_angle = (angle_index - 135) * (pi_ / 180);
             }
             //publish the path message
             micros_mars_task_alloc::PathPtr path_ptr(new micros_mars_task_alloc::Path);
             path_ptr -> path_angle = turn_angle;
             path_ptr -> path_distance = max_distance;
             pub_.publish(path_ptr);
             cout << "Look module is working." << endl; 
             start_look_ = false;
        }
        else
            cout << "Look module is waiting for the trigger message." << endl;
    }

    void Look::callback_1(const std_msgs::BoolConstPtr & msg)
    {
        start_look_ = msg->data; 
        /*
        if(start_look_)
        {
            ros::Duration(0.5).sleep();
            start_look_ = false;
        }
        */
    }
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Look, nodelet::Nodelet)
