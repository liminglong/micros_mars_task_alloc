/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: Aug. 31th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <micros_mars_task_alloc/MoveBaseAction.h>
#include <micros_mars_task_alloc/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

namespace micros_mars_task_alloc {
using namespace std;

class RobotPatrol : public nodelet::Nodelet
{
public:
    RobotPatrol():goal_count_(0), goal_time_interval_(10.0){}
    virtual ~RobotPatrol(){}
    virtual void onInit();
    void timerCallback(const ros::TimerEvent&);
private:
    typedef actionlib::SimpleActionClient<micros_mars_task_alloc::MoveBaseAction> Client;
    boost::shared_ptr<Client> client_;    
    ros::Timer timer_;
    ros::NodeHandle nh_;
    
    float goal_0_pos_x_, goal_0_pos_y_, goal_0_ori_z_, goal_0_ori_w_;//goal 0
    float goal_1_pos_x_, goal_1_pos_y_, goal_1_ori_z_, goal_1_ori_w_;//goal 1
    float goal_2_pos_x_, goal_2_pos_y_, goal_2_ori_z_, goal_2_ori_w_;//goal 2
    float goal_3_pos_x_, goal_3_pos_y_, goal_3_ori_z_, goal_3_ori_w_;//goal 3

    micros_mars_task_alloc::MoveBaseGoal goal_0_;
    micros_mars_task_alloc::MoveBaseGoal goal_1_;
    micros_mars_task_alloc::MoveBaseGoal goal_2_;
    micros_mars_task_alloc::MoveBaseGoal goal_3_;
    
    int goal_count_;
    double goal_time_interval_;//the time interval of patroling goal to goal
};

void RobotPatrol::onInit()
{
    nh_ = getMTPrivateNodeHandle();//Private multi-threaded nodehandle
    std::string goal_topic;//the goal is sent to the robot on this topic    
    nh_.param<std::string>("goal_topic", goal_topic, "default_topic");
    client_.reset(new actionlib::SimpleActionClient<micros_mars_task_alloc::MoveBaseAction>(goal_topic, true));
    double x0, y0, z0, w0, x1, y1, z1, w1, x2, y2, z2, w2, x3, y3, z3, w3;
    //get the goal 0 from the parameter server
    nh_.param<double>("goal_0_pos_x", x0, 0.0);//0.0 represents the default position
    nh_.param<double>("goal_0_pos_y", y0, 0.0);
    nh_.param<double>("goal_0_ori_z", z0, 0.0);
    nh_.param<double>("goal_0_ori_w", w0, 0.0);
    //goal 1
    nh_.param<double>("goal_1_pos_x", x1, 0.0);//0.0 represents the default position
    nh_.param<double>("goal_1_pos_y", y1, 0.0);
    nh_.param<double>("goal_1_ori_z", z1, 0.0);
    nh_.param<double>("goal_1_ori_w", w1, 0.0);
    //goal 2
    nh_.param<double>("goal_2_pos_x", x2, 0.0);//0.0 represents the default position
    nh_.param<double>("goal_2_pos_y", y2, 0.0);
    nh_.param<double>("goal_2_ori_z", z2, 0.0);
    nh_.param<double>("goal_2_ori_w", w2, 0.0);
    //goal 3
    nh_.param<double>("goal_3_pos_x", x3, 0.0);//0.0 represents the default position
    nh_.param<double>("goal_3_pos_y", y3, 0.0);
    nh_.param<double>("goal_3_ori_z", z3, 0.0);
    nh_.param<double>("goal_3_ori_w", w3, 0.0);
    
    goal_0_pos_x_ = (float)x0;
    goal_0_pos_y_ = (float)y0;
    goal_0_ori_z_ = (float)z0;
    goal_0_ori_w_ = (float)w0;
    
    goal_1_pos_x_ = (float)x1;
    goal_1_pos_y_ = (float)y1;
    goal_1_ori_z_ = (float)z1;
    goal_1_ori_w_ = (float)w1;
    
    goal_2_pos_x_ = (float)x2;
    goal_2_pos_y_ = (float)y2;
    goal_2_ori_z_ = (float)z2;
    goal_2_ori_w_ = (float)w2;
    
    goal_3_pos_x_ = (float)x3;
    goal_3_pos_y_ = (float)y3;
    goal_3_ori_z_ = (float)z3;
    goal_3_ori_w_ = (float)w3;
    
    goal_0_.target_pose.header.frame_id = "map";
    goal_0_.target_pose.pose.position.x = goal_0_pos_x_;
    goal_0_.target_pose.pose.position.y = goal_0_pos_y_;
    goal_0_.target_pose.pose.position.z = 0.0;
    goal_0_.target_pose.pose.orientation.x = 0.0;
    goal_0_.target_pose.pose.orientation.y = 0.0;
    goal_0_.target_pose.pose.orientation.z = goal_0_ori_z_;
    goal_0_.target_pose.pose.orientation.w = goal_0_ori_w_;

    goal_1_.target_pose.header.frame_id = "map";
    goal_1_.target_pose.pose.position.x = goal_1_pos_x_;
    goal_1_.target_pose.pose.position.y = goal_1_pos_y_;
    goal_1_.target_pose.pose.position.z = 0.0;
    goal_1_.target_pose.pose.orientation.x = 0.0;
    goal_1_.target_pose.pose.orientation.y = 0.0;
    goal_1_.target_pose.pose.orientation.z = goal_1_ori_z_;
    goal_1_.target_pose.pose.orientation.w = goal_1_ori_w_;

    goal_2_.target_pose.header.frame_id = "map";
    goal_2_.target_pose.pose.position.x = goal_2_pos_x_;
    goal_2_.target_pose.pose.position.y = goal_2_pos_y_;
    goal_2_.target_pose.pose.position.z =  0.0;
    goal_2_.target_pose.pose.orientation.x = 0.0;
    goal_2_.target_pose.pose.orientation.y = 0.0;
    goal_2_.target_pose.pose.orientation.z = goal_2_ori_z_;
    goal_2_.target_pose.pose.orientation.w = goal_2_ori_w_;

    goal_3_.target_pose.header.frame_id = "map";
    goal_3_.target_pose.pose.position.x = goal_3_pos_x_;
    goal_3_.target_pose.pose.position.y = goal_3_pos_y_;
    goal_3_.target_pose.pose.position.z = 0.0;
    goal_3_.target_pose.pose.orientation.x = 0.0;
    goal_3_.target_pose.pose.orientation.y = 0.0;
    goal_3_.target_pose.pose.orientation.z = goal_3_ori_z_;
    goal_3_.target_pose.pose.orientation.w = goal_3_ori_w_;
    
    timer_ = nh_.createTimer(ros::Duration(goal_time_interval_), &RobotPatrol::timerCallback, this);//10.0 is a patrol cycle.
}

void RobotPatrol::timerCallback(const ros::TimerEvent&)
{
     switch(goal_count_)
     {
         case 0:
         {
             client_ -> sendGoal(goal_0_);
             break;    
         }
         case 1:
         {
             client_ -> sendGoal(goal_1_);
             break;
         }
         case 2:
         {
             client_ -> sendGoal(goal_2_);
             break;             
         }
         case 3:
         {
             client_ -> sendGoal(goal_3_);
             break;
         }
         default:
             std::cout << "ERROR!" << endl;
     }
     goal_count_ += 1;
     if(goal_count_ == 4)
        goal_count_ = 0;
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::RobotPatrol, nodelet::Nodelet)
