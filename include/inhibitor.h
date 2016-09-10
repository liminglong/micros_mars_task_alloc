/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: June 4th, 2016
*/
#ifndef INHIBITOR_H_
#define INHIBITOR_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>

namespace micros_mars_task_alloc {

template<typename UpperMsgType, typename LowerMsgType>
class Inhibitor : public nodelet::Nodelet
{
    //typedef typename boost::shared_ptr<MsgType> TMsgPtr;
    //typedef typename boost::shared_ptr<const T> TConstPtr;
public:
    Inhibitor(){}
    virtual ~Inhibitor(){}
    virtual void onInit()
    {
        nh_ = getPrivateNodeHandle();
        std::cout << "Initialising nodelet inhibitor ..." << std::endl;        
        // resolve node(let) name
        /*
        std::string name = nh_.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos + 1);
        std::cout << "Initialising nodelet... [" << name << "]"<< std::endl;
        */
        // get the parameters
        if (!(nh_.getParam("upper_topic", upper_topic_)))
        {
            std::cout << "Fail to get the parameter upper_topic." << std::endl;
            return;
        }
        if (!(nh_.getParam("lower_topic", lower_topic_)))
        {
            std::cout << "Fail to get the parameter lower_topic" << std::endl;
            return;
        }
        if (!(nh_.getParam("output_topic",output_topic_)))
        {
            std::cout << "Fail to get the parameter output_topic" << std::endl;
            return;
        }
        nh_.param<double>("time_duration", time_duration_, 1.0);
        std::cout << "inhibitor hello1" << std::endl;
        upper_msg_sub_ = nh_.subscribe(upper_topic_, 10, &Inhibitor::upperCB, this);
        std::cout << "inhibitor hello2" << std::endl;
        lower_msg_sub_ = nh_.subscribe(lower_topic_, 10, &Inhibitor::lowerCB, this);
        std::cout << "inhibitor hello3" << std::endl;
        pub_ = nh_.advertise<LowerMsgType>(output_topic_, 10);//the typename MsgType represents the type of the suppressed messages
        std::cout << "inhibitor was loaded successfully." << std::endl;
        
    }
    //void upperCB(boost::shared_ptr<MsgType const>& msg);
    void upperCB(const boost::shared_ptr<const UpperMsgType>& msg)
    {
        begin_ = ros::Time::now().toSec();
        /*
        pub_ = nh_.advertise<MsgType>(output_topic_, 1000);
        pub_.publish(msg);
        */
        std::cout << "The upper layer are working." << std::endl;
    };

    //void lowerCB(boost::shared_ptr<MsgType const>& msg);  
    void lowerCB(const boost::shared_ptr<const LowerMsgType>& msg)
    {
        end_ = ros::Time::now().toSec();
        if(end_-begin_ > time_duration_)
        {
            //pub_ = nh_.advertise<MsgType>(output_topic_, 1000);
            pub_.publish(msg);
            std::cout << "The upper layer has broken down, the lower layer messages output successfully." << std::endl;
        }
    };  
    
private:
    ros::NodeHandle nh_;
    std::string upper_topic_, lower_topic_, output_topic_;
    ros::Subscriber upper_msg_sub_, lower_msg_sub_;
    ros::Publisher pub_;
    double begin_;//to calculate the time duration
    double end_;//to calculate the time duration
    double time_duration_;

};

}//namespace micros_mars_task_alloc
#endif
