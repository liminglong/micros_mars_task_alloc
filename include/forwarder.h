/*
 * Author:      Minglong Li
 * Affiliation: State Key Laboratory of High Performance Computing (HPCL)
 *              College of Computer, National University of Defense Technology
 * Email:       minglong_l@163.com
 * Created on:  Aug. 31th, 2016
 *
 * Description: The primary mechanism of ALLIANCE model enabling a robot to select a high-
 *              level function to activate is the 'motivational behavior'. This module named 
 *              'forwarder' that connects the output of each 'motivational behavior' with 
 *              the output of its behavior set indicates that a motivational behavior either 
 *              allows all or none of the outputs of its behavior set to pass through to the 
 *              robot's actuators. This module can forward two message flows at most.
 */

#ifndef FORWARDER_H_
#define FORWARDER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

namespace micros_mars_task_alloc {

using namespace std;

template<typename MsgType_0, typename MsgType_1 = std_msgs::Bool>
class Forwarder : public nodelet::Nodelet
{

public:
    Forwarder():activating_flag_(false){}
    virtual ~Forwarder(){}
    virtual void onInit()
    {
        nh_ = getMTPrivateNodeHandle();

        nh_.param<std::string>("activating_topic", activating_topic_, "activating_topic");//The third parameter is the default value
        nh_.param<std::string>("input_topic_0", input_topic_0_, "input_topic_0");
        nh_.param<std::string>("input_topic_1", input_topic_1_, "input_topic_1");
        nh_.param<std::string>("output_topic_0", output_topic_0_, "output_topic_0");
        nh_.param<std::string>("output_topic_1", output_topic_1_, "output_topic_1");
        
        sub_0_ = nh_.subscribe(input_topic_0_, 10, &Forwarder::callback_0, this);
        sub_1_ = nh_.subscribe(input_topic_1_, 10, &Forwarder::callback_1, this);
        sub_2_ = nh_.subscribe(activating_topic_, 10, &Forwarder::activating_callback, this);
        
        pub_0_ = nh_.advertise<MsgType_0>(output_topic_0_, 10);
        pub_1_ = nh_.advertise<MsgType_1>(output_topic_1_, 10);
    } 
    
    void callback_0(const boost::shared_ptr<const MsgType_0>& msg)
    {
        cout << "callback_0 start!" << endl;
        if (activating_flag_ == true)
        {
            pub_0_.publish(msg);
        }
    }

    void callback_1(const boost::shared_ptr<const MsgType_1>& msg)
    {
        cout << "callback_1 start!" << endl;
        activating_flag_ = msg -> data;
        if (activating_flag_ == true)
        {
            pub_1_.publish(msg);
        }
    }

    void activating_callback(const std_msgs::BoolConstPtr & msg)
    {
        cout << "activating_callback start!" << endl;
        activating_flag_ = msg -> data;
    }
    
private:
    std::string activating_topic_;
    std::string input_topic_0_;
    std::string input_topic_1_;
    std::string output_topic_0_;
    std::string output_topic_1_;
    
    bool activating_flag_;
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_0_, sub_1_, sub_2_;
    ros::Publisher pub_0_, pub_1_;
};


}//namespace micros_mars_task_alloc
#endif
