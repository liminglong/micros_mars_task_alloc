/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 16th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <limits> 

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

namespace micros_mars_task_alloc {
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

class Collide : public nodelet::Nodelet
{
public:
    Collide(): halt_flag_(false), n_inf_(-std::numeric_limits<float>::infinity()), maximum_queue_size_(10){}
    ~Collide(){}
    virtual void onInit();
    void callback(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, const RangeConstPtr& msg_10, const RangeConstPtr& msg_11);
                                                  
private:
    //every force has a range and an angle.
    bool halt_flag_;
    float n_inf_;
    int maximum_queue_size_;//control the queue size of the message filters
    typedef sync_policies::ApproximateTime<Range, Range, Range, Range, Range> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    
    //Because message_filters support 9 topics at most, use two filters and two callbacks to handle the sonar messages.
    boost::shared_ptr<ApproximateSync> approximate_sync_;    

    message_filters::Subscriber<Range> sub_0_, sub_1_, sub_2_, sub_10_, sub_11_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;

};

void Collide::onInit()
{
    nh_ = getPrivateNodeHandle();
    cout << "Initialising nodelet ..." << endl;        

    //The topics should be absolute.
    sub_0_.subscribe(nh_, "/robot0/sonar_0", 1);
    sub_1_.subscribe(nh_, "/robot0/sonar_1", 1);
    sub_2_.subscribe(nh_, "/robot0/sonar_2", 1);
    sub_10_.subscribe(nh_, "/robot0/sonar_10", 1);
    sub_11_.subscribe(nh_, "/robot0/sonar_11", 1);   
    
    pub_ = nh_.advertise<std_msgs::Bool>("halt", 10);
    
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(maximum_queue_size_), sub_0_, sub_1_, sub_2_, sub_10_, sub_11_) );
    approximate_sync_->registerCallback(boost::bind(&Collide::callback, this, _1, _2, _3, _4, _5));
}

void Collide::callback(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, const RangeConstPtr& msg_10, const RangeConstPtr& msg_11)
{
    
    if ((msg_0->range!=n_inf_)&&(msg_1->range!=n_inf_)&&(msg_2->range!=n_inf_)&&(msg_10->range!=n_inf_)&&(msg_11->range!=n_inf_))
        halt_flag_ = false;
    else
        halt_flag_ = true;
    
    std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
    bool_ptr->data = halt_flag_;
    pub_.publish(bool_ptr);
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Collide, nodelet::Nodelet)
