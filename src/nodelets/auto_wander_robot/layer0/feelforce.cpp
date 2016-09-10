/*
Author: Minglong Li
Affiliation: State Key Laboratory of High Performance Computing (HPCL)
             College of Computer, National University of Defense Technology
Email: minglong_l@163.com
Created on: July 11th, 2016
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <limits> 
#include <iostream>
#include <pluginlib/class_list_macros.h>

#include <vector>
#include <algorithm>
#include <functional> 
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Range.h>
#include <micros_mars_task_alloc/Force.h>

namespace micros_mars_task_alloc {
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

class Feelforce : public nodelet::Nodelet
{
public:
    Feelforce(): maximum_queue_size_(10), min_range_(0.3), max_range_(3.0), pi_(std::acos(-1)){}
    ~Feelforce(){}

    virtual void onInit();

    
   //Because message_filters support 9 topics at most, use two filters and two callbacks to handle the sonar messages.
    void callback_0(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, 
                    const RangeConstPtr& msg_3, const RangeConstPtr& msg_4, const RangeConstPtr& msg_5); 
    void callback_1(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, 
                    const RangeConstPtr& msg_3, const RangeConstPtr& msg_4, const RangeConstPtr& msg_5);                                              

    void timerCallback(const ros::TimerEvent&);
    //This struct is used to sum the feelforce and has higher efficiency.    
    typedef struct force
    {
        float range;
        float angle;
    }Force;
    //The synchronizing policy is approximate.
private:
    //every force has a range and an angle.

    typedef sync_policies::ApproximateTime<Range, Range, Range, Range, Range, Range> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    
    //Because message_filters support 9 topics at most, use two filters and two callbacks to handle the sonar messages.
    boost::shared_ptr<ApproximateSync> approximate_sync_1_;    
    boost::shared_ptr<ApproximateSync> approximate_sync_2_;
    
    float pi_;
    int maximum_queue_size_;//control the queue size of the message filters
    float min_range_;//if the sensed distance is smaller than the minimum distance, assign it the parameter
    float max_range_;//if the sensed distance is larger than the max distance, assign it the parameter

    float r_[12];//store the range of the 12 sensor messages.

    message_filters::Subscriber<Range> sub_0_, sub_1_, sub_2_, sub_3_, sub_4_, sub_5_, sub_6_, sub_7_, sub_8_, sub_9_, sub_10_, sub_11_;
    ros::Timer timer_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;

};

void Feelforce::onInit()
{
    nh_ = getPrivateNodeHandle();
    cout << "Initialising nodelet ..." << endl;        

    //The topics should be absolute.
    sub_0_.subscribe(nh_, "/robot0/sonar_0", 1);
    sub_1_.subscribe(nh_, "/robot0/sonar_1", 1);
    sub_2_.subscribe(nh_, "/robot0/sonar_2", 1);
    sub_3_.subscribe(nh_, "/robot0/sonar_3", 1);   
    sub_4_.subscribe(nh_, "/robot0/sonar_4", 1);
    sub_5_.subscribe(nh_, "/robot0/sonar_5", 1);
    sub_6_.subscribe(nh_, "/robot0/sonar_6", 1);
    sub_7_.subscribe(nh_, "/robot0/sonar_7", 1);   
    sub_8_.subscribe(nh_, "/robot0/sonar_8", 1);
    sub_9_.subscribe(nh_, "/robot0/sonar_9", 1);
    sub_10_.subscribe(nh_, "/robot0/sonar_10", 1);
    sub_11_.subscribe(nh_, "/robot0/sonar_11", 1);   
    
    approximate_sync_1_.reset( new ApproximateSync(ApproximatePolicy(maximum_queue_size_), sub_0_, sub_1_, sub_2_, sub_3_, sub_4_, sub_5_ ) );
    approximate_sync_1_->registerCallback(boost::bind(&Feelforce::callback_0, this, _1, _2, _3, _4, _5, _6));
    
    approximate_sync_2_.reset( new ApproximateSync(ApproximatePolicy(maximum_queue_size_), sub_6_, sub_7_, sub_8_, sub_9_, sub_10_, sub_11_ ) );
    approximate_sync_2_->registerCallback(boost::bind(&Feelforce::callback_1, this, _1, _2, _3, _4, _5, _6));    
    
    timer_ = nh_.createTimer(ros::Duration(0.1), &Feelforce::timerCallback, this);

}

void Feelforce::callback_0(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, 
                           const RangeConstPtr& msg_3, const RangeConstPtr& msg_4, const RangeConstPtr& msg_5)
{
    //cout << "callback_0 start!" << endl;
    r_[0] = msg_0 -> range;
    r_[1] = msg_1 -> range;
    r_[2] = msg_2 -> range;
    r_[3] = msg_3 -> range;    
    r_[4] = msg_4 -> range;
    r_[5] = msg_5 -> range;
}

void Feelforce::callback_1(const RangeConstPtr& msg_0, const RangeConstPtr& msg_1, const RangeConstPtr& msg_2, 
                           const RangeConstPtr& msg_3, const RangeConstPtr& msg_4, const RangeConstPtr& msg_5)
{
    //cout << "callback_1 start" << endl;
    r_[6] = msg_0 -> range;
    r_[7] = msg_1 -> range;
    r_[8] = msg_2 -> range;
    r_[9] = msg_3 -> range;    
    r_[10] = msg_4 -> range;
    r_[11] = msg_5 -> range;
}

void Feelforce::timerCallback(const ros::TimerEvent&)
{
    vector<Force> vec;
    
    float rectangular_coordinates_sum_x = 0;
    float rectangular_coordinates_sum_y = 0;
    for (int i = 0; i < 12; i++)
    {
        Force a;
        /*
        Fixed distance rangers only output -Inf or +Inf.
        -Inf represents a detection within fixed distance.
        (Detection too close to the sensor to quantify)
        +Inf represents no detection within the fixed distance.
        (Object out of range)
        */
        if (r_[i] == std::numeric_limits<float>::infinity()) 
            a.range = max_range_;
        else if (r_[i] == -std::numeric_limits<float>::infinity())
            a.range = min_range_;
        else
            a.range = r_[i];

        a.angle = i * 30.0;
        vec.push_back(a);
    }
    //variable capture: notice that the lambda function uses the variable in the outside block, and capture the this ptr.
    for_each(vec.begin(), vec.end(), [&, this](const Force& val)->void
    {
        //cout << "val.range: " << val.range << endl; 
        //cout << "std::cos(pi_ * val.angle / 180.0) :" << std::cos(pi_ * val.angle / 180.0) << endl;
        //cout << "(1/(val.range*val.range)) * std::cos(pi_ * val.angle / 180.0)" << (1/(val.range*val.range)) * std::cos(pi_ * val.angle / 180.0) << endl;
        rectangular_coordinates_sum_x += (1/(val.range*val.range)) * std::cos(pi_ * val.angle / 180.0);//TODO:pi?
        rectangular_coordinates_sum_y += (1/(val.range*val.range)) * std::sin(pi_ * val.angle / 180.0);        
    });
    rectangular_coordinates_sum_x = (-1) * rectangular_coordinates_sum_x;
	rectangular_coordinates_sum_y = (-1) * rectangular_coordinates_sum_y;
    //cout << "rectangular_coordinates_sum_x: " << rectangular_coordinates_sum_x << endl;
    //cout << "rectangular_coordinates_sum_y: " << rectangular_coordinates_sum_y << endl;
	//pub the coordinates_sum message.
	micros_mars_task_alloc::ForcePtr force_ptr(new micros_mars_task_alloc::Force);
    //devided by 12 to avoid the resulted distance being too large
    force_ptr -> magnitude = float( sqrt(double(rectangular_coordinates_sum_x * rectangular_coordinates_sum_x) + double(rectangular_coordinates_sum_y * rectangular_coordinates_sum_y))/12.0);
	force_ptr -> direction = float( fmod((atan2(double(rectangular_coordinates_sum_y), double(rectangular_coordinates_sum_x)) + 2 * double(pi_)) , (2 * double(pi_))) );
	
	//cout << "force_ptr -> magnitude: " << force_ptr -> magnitude << endl;
	//cout << "force_ptr -> direction: " << force_ptr -> direction << endl;
	
	pub_ = nh_.advertise<micros_mars_task_alloc::Force>("force", 10);//This topic is a relative topic. Because of getPrivateNodehandle(), nodelet name feelforce will be added as the prefix.
	pub_.publish(force_ptr);
}
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::Feelforce, nodelet::Nodelet)
