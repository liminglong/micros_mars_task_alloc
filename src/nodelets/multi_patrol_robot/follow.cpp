/*************************************************************************
	> File Name: robot_detect.cpp
	> Author: Minglong Li
	> Mail: minglong_l@163.com
	> Created on: Aug. 15th, 2016
 ************************************************************************/
 
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

/*
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
*/
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace micros_mars_task_alloc{
    using namespace std;
    using namespace message_filters;
    class RobotFollow : public nodelet::Nodelet
    {
    public:
        RobotFollow(): maximum_queue_size_(10), pi_(std::acos(-1)), follow_distance_(0.5), linear_velocity_(0.3), angular_velocity_(0.5){}
        virtual ~RobotFollow(){}

        virtual void onInit();
        
        void callback(const nav_msgs::OdometryConstPtr & msg_0, const nav_msgs::OdometryConstPtr & msg_1);
        
        
    private:    
        ros::NodeHandle nh_;
        
        typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> ApproximatePolicy;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        
        boost::shared_ptr<ApproximateSync> approximate_sync_;
        
        message_filters::Subscriber<nav_msgs::Odometry> sub_0_, sub_1_;
        float pi_, follow_distance_, linear_velocity_, angular_velocity_; 
        int maximum_queue_size_;//control the queue size of the message filters
        ros::Publisher pub_;
    };

    void RobotFollow::onInit()
    {
        nh_ = getMTPrivateNodeHandle();
        cout << "Initialising nodelet robot follow" << endl;
        
        sub_0_.subscribe(nh_, "/robot_0/base_pose_ground_truth", 1);
        sub_1_.subscribe(nh_, "/robot_3/base_pose_ground_truth", 1);
        
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(maximum_queue_size_), sub_0_, sub_1_) );
        approximate_sync_->registerCallback(boost::bind(&RobotFollow::callback, this, _1, _2));
        pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);//This is a relative topic, the prefix look will be added automaically.
    }

    void RobotFollow::callback(const nav_msgs::OdometryConstPtr & msg_0, const nav_msgs::OdometryConstPtr & msg_1)
    {
        cout << "callback start!" << endl;
        float x, y, z, w;//x, y, z and w represents the orientation of the follower(tetracyclic coordinates).
        float follower_pose_x;//position of the follower.
        float follower_pose_y;//postion of the follower.
        float intruder_pose_x;//position of the intruder.
        float intruder_pose_y;//position of the intruder.

        float leader_direction;//the direction of the intruder
        float follower_orientation;
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist);
        
        x = msg_0->pose.pose.orientation.x;
        y = msg_0->pose.pose.orientation.y;
        z = msg_0->pose.pose.orientation.z;
        w = msg_0->pose.pose.orientation.w;
        
        follower_pose_x = msg_0->pose.pose.position.x;
        follower_pose_y = msg_0->pose.pose.position.y;
        
        intruder_pose_x = msg_1->pose.pose.position.x;
        intruder_pose_y = msg_1->pose.pose.position.y;
        
        leader_direction = float(atan2(double(intruder_pose_y - follower_pose_y), double(intruder_pose_x - follower_pose_x)));
        if ((leader_direction > (-1)*pi_ || leader_direction == (-1) * pi_) && (leader_direction < 0))
            leader_direction = 2 * pi_ + leader_direction;
        
        follower_orientation = float(atan2(double(2*(w*z+x*y)), double(1-2*(y*y+z*z))));
        if ((follower_orientation > -1*pi_ || follower_orientation == -1*pi_) && (follower_orientation < 0))
            follower_orientation = 2*pi_ + follower_orientation;
        
        if (((follower_pose_x - intruder_pose_x)*(follower_pose_x - intruder_pose_x) + (follower_pose_y - intruder_pose_y)*(follower_pose_y - intruder_pose_y)) > follow_distance_*follow_distance_)
        {
            if (follower_orientation < leader_direction)
            {
                cmd->linear.x = linear_velocity_;
                cmd->angular.z = angular_velocity_;
                pub_.publish(cmd);   
            }
            else
            {
                cmd->linear.x = linear_velocity_;
                cmd->angular.z = -1 * angular_velocity_;
                pub_.publish(cmd);   
            }
        }
        else
        {
            cmd->linear.x = 0.0;
            cmd->angular.z = 0.0;
            pub_.publish(cmd);   
        }
    }

}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::RobotFollow, nodelet::Nodelet)
