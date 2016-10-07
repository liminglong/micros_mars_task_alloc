/*************************************************************************
	> File Name: robot_detect.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
	> Created on: Aug. 12th, 2016
 ************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pluginlib/class_list_macros.h>

namespace micros_mars_task_alloc{
    using namespace std;
    class RobotDetect : public nodelet::Nodelet
    {
    public:
        RobotDetect(){}
        virtual ~RobotDetect(){}

        virtual void onInit();
        
        void callback(const sensor_msgs::ImageConstPtr & msg);

    private:    
        ros::NodeHandle nh_;
        ros::Subscriber sub_; 
        ros::Publisher pub_;
        std::string output_topic_;
        //std::bool detected_flag;
    };
    void RobotDetect::onInit()
    {
        nh_ = getMTPrivateNodeHandle();
        if (!(nh_.getParam("output_topic", output_topic_)))
        {
            std::cout << "Fail to get the parameter output_topic." << std::endl;
            return;
        } 
        pub_ = nh_.advertise<std_msgs::Bool>(output_topic_, 10);                  
        sub_ = nh_.subscribe("/robot_0/image", 10, &RobotDetect::callback, this);//the topic here does not change
    }
    void RobotDetect::callback(const sensor_msgs::ImageConstPtr & msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat_<cv::Vec3b>::iterator it = cv_ptr->image.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::iterator itend = cv_ptr->image.end<cv::Vec3b>();
        bool flag = false;
        for(;it != itend; ++it)
        {
            if((*it)[0]==0 && (*it)[1]==0 && (*it)[2]==255)
            {
                 flag = true;
            }
        }
        if(flag)
        {
            std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
            bool_ptr -> data = true;
            pub_.publish(bool_ptr);
        }
    }
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::RobotDetect, nodelet::Nodelet)
