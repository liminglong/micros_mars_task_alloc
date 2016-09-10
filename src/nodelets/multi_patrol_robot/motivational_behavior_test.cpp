/*************************************************************************************************************************
 * Author:      Minglong Li
 * Affiliation: State Key Laboratory of High Performance Computing (HPCL)
 *              College of Computer, National University of Defense Technology
 * Email:       minglong_l@163.com
 * Created on:  Sep. 5th, 2016
 *
 * Description: In ALLIANCE, the ability for robots to respond to unexpected events, robot failures, and so forth,
 *              is provided through the use of motivations. These motivations are designed to allow robot team members
 *              to perform tasks only as long as they demonstrate their ability to have the desired effect on the world.
 *              This differs from the commonly used technique for task allocation that begins with breaking down the 
 *              mission (or part of the mission) into subtasks, and then computing the "optimal" robot-to-task mapping 
 *              based upon agent skill levels, with little recourse for robot failures after the allocation has occurred.                
 **************************************************************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <micros_mars_task_alloc/Heartbeat.h>

#include <vector>

namespace micros_mars_task_alloc{
    using namespace std;
    
    template<typename VS_MsgType>//the sensory_feedback message type of the 'Virtual Sensory' messages.
    class MotivationalBehavior : public nodelet::Nodelet
    {
    public:
        MotivationalBehavior(): motivation_(0), sensory_feedback_(1), sensory_feedback_exist_(false), maximum_time_step_(1000), 
                                delta_fast_(10), delta_slow_(0.01), threshold_(1000), impatience_reset_flag_(true){}
        virtual ~MotivationalBehavior(){}

        virtual void onInit()
        {
            nh_ = getMTPrivateNodeHandle();
            
            if (!(nh_.getParam("robot_number", robot_number_)))
            {
                std::cout << "Fail to get the parameter robot_number." << std::endl;
                return;
            }           
            
            if (!(nh_.getParam("behavior_set_number", behavior_set_number_)))
            {
                std::cout << "Fail to get the parameter behavior_set_number." << std::endl;
                return;
            }
            
            //initialise the vector heartbeat_, the row number is robot_number_ and the column number is maximum_time_step_.
            heartbeat_.resize(robot_number_);
            for (int i = 0; i < heartbeat_.size(); i++)
            {
                heartbeat_.resize(maximum_time_step_);
            }
            
            for (int i = 0; i < heartbeat_.size(); i++)
            {
                for (int j = 0; j < heartbeat_[0].size(); j++)
                {
                    heartbeat_[i][j] = false;
                }
            }
            
            //initialise the heartbeat_count
            heartbeat_count_.resize(robot_number_)
            for (int i = 0; i < heartbeat_count_.size(); i++)
            {
                heartbeat_count_[i] = 0;
            }
            
            
            //initialise the vector comm_received and comm_received_timestamp_, the default value of the elements in this vector is set to 0.
            for (int i = 0; i < robot_number_; i++)
            {
                comm_received_.push_back(0);
                comm_received_timestamp_.push_back(0);
            }
            
            
            
            main_periodic_timer_ = nh_.createTimer(ros::Duration(0.1), &MotivationalBehavior::main_periodic_callback, this);//control the main logic.
            
            sub_0_ = nh_.subscribe(sensory_feedback_topic_, 10, &MotivationalBehavior::sensory_feedback_callback, this);//sub_0_ is used to hangle the inter-robot communicating messages.
            sub_1_ = nh_.subscribe(inter_robot_comm_topic_, 10, &MotivationalBehavior::inter_robot_comm_callback, this);//sub_1_ is used to handle the intra-robot communicating messages.
            sub_2_ = nh_.subscribe(intra_robot_comm_topic_, 10, &MotivationalBehavior::intra_robot_comm_callback, this);
        }

        void sensory_feedback_callback(const boost::shared_ptr<const VS_MsgType> & msg)
        {
            cout << "sensory_feedback_callback start!" << endl;
            sensory_feedback_exist_ = true;
            sensory_feedback_timestamp_ = ros::Time::now().toSec();//if the sensory_feedback is received, even only once, this flag paramter will be set to true.
            sensory_feedback_ = 1;
        }

        void main_periodic_callback(const ros::TimerEvent&)
        {
            double update_time;
            update_time = ros::Time::now().toSec();
            if ((sensory_feedback_exist_) && (update_time - sensory_feedback_timestamp_ > 1.0))//The time duration for the sensory feedback is 1.0 sensond
            {
                sensory_feedback_ = 0;
            }
            impatience_calc();
        }

        void impatience_calc()// impatience_reset_ is included in it.
        {
            bool delta_fast_flag = true;
            
            //impatience reset: when this robot hears about another robot performing this behavior set, and check only once.
            if(impatience_reset_flag_)
            {
                for (int i = 0; i < robot_number_; i ++)
                {
                    if (  (heartbeat_[i][ heartbeat_count_[i] ] == true)
                        &&(heartbeat_[i][ heartbeat_count_[i]-1 ] == true)
                        &&(heartbeat_[i][ heartbeat_count_[i]-2 ] == true) 
                        && (i != this_robot_ID_))
                    {                    
                        impatience_ = 0;
                        impatience_reset_flag_ = false;
                    }
                }
            }
            
            //gama is 3, phi is 5.  
            for (int i = 0; i < robot_number_; i++)//i is robot_ID
            {
                //because heartbeat_ is a ring_queue, we need to handle the index number of ring queue
                int m = (heartbeat_count_[i] - 3 + maximum_time_step_) % maximum_time_step_;
                int n = (heartbeat_count_[i] - 5 + maximum_time_step_) % maximum_time_step_;
            
                if(    (heartbeat_[i][ heartbeat_count_[i] ] == true)
                    && (heartbeat_[i][ heartbeat_count_[i] - m ] == true) 
                    && (heartbeat_[i][ heartbeat_count_[i] - n ] == false) 
                    && (i != this_robot_ID_))
                {
                    delta_fast_flag = false;
                }
                else
                {
                    delta_fast_flag = true;
                }
            }
            
            if(delta_fast_flag)
            {
                impatience_ = impatience_ + delta_fast_;   
                if(impatience_ > threshold_)
                    impatience_ = threshold_;
            }
            else         
            {    
                impatience_ = impatience_ + delta_slow_;
                if(impatience_ > threshold_)
                    impatience_ = threshold_;
            }  
            
                              
        }


        void acquiescence_calc()
        {
            //TODO: to be implemented.
        }

        void inter_robot_comm_callback(const micros_mars_task_alloc::HeartbeatConstPtr & msg)
        {
            cout << "inter_robot_comm_callback start!" << endl;
            if( (msg->robot_ID != this_robot_ID_)&&(msg->behavior_set_ID == this_behavior_set_ID) )
            {
                heartbeat_[msg->robot_ID][ heartbeat_count_[msg->robot_ID] ] = msg->heartbeat;
                heartbeat_count_[msg->robot_ID]+=1;
                if(heartbeat_count_[msg->robot_ID] == maximum_time_step_)
                {
                    heartbeat_count_[msg->robot_ID] = 0;
                }                
            }
        }

        void intra_robot_comm_callback(const micros_mars_task_alloc::HeartbeatConstPtr & msg)
        {
            cout << "intra_robot_comm_callback start!" << endl;
            if((msg->heartbeat==true) && (msg->robot_ID == this_robot_ID_) && (msg->behavior_set_ID != this_behavior_set_ID_))//any other behavior set in this robot is activated.
            {
                activity_suppression_ = 0;
            }
            else//none of other behavior set in this robot is activated.
            {
                activity_suppression_ = 1;
            }
        }

    private:    
        ros::NodeHandle nh_;
        ros::Timer main_periodic_timer_;
        ros::Subscriber sub_0_;
        ros::Subscriber sub_1_;
        ros::Subscriber sub_2_;
        ros::Publisher pub_;
        
        int this_robot_ID_;//the ID of this robot, to be set by the paramters in the launch file.
        int this_behavior_set_ID_;//the ID of this behavior set in this robot, to be set by the parameters in the launch file.
        int behavior_set_number_;//the total number of the behavior sets, which is set by the local parameters
        int robot_number_;//the total number of the robots, which is set by the local parameters 
        
        int maximum_time_step_;

        std::string sensory_feedback_topic_;
        std::string inter_robot_comm_topic_;
        std::string intra_robot_comm_topic_;
    
        //Some parameters used to calculate the motivation, TODO: these may be used as local parameters.
        double impatience_;
        double acquiescence_;
        double sensory_feedback_;//TODO: the default value of sensory feedback is 1.
        double sensory_feedback_timestamp_;

        
        bool sensory_feedback_exist_;//Not all motivational behavior exist the sensory feedback item, the default value of this parameter is false, which means it does not exist.
        double activity_suppression_;//if any other behavior set in this robot is activated, the motivation of this behavior set is set to 0. 
        double impatience_reset_;
        
        double delta_slow_;//when another robot is doing this work, the increasing velocity of impatience in this motivational behavior is delta_slow_
        double delta_fast_;//when none of the robots is doing this work, the increasing velocity of impatience in this motivational behavior is delta_fast_
        double threshold_;
        
        bool impatience_reset_check_flag_;
        
        vector<int> heartbeat_count_;

        vector< vector<bool> > heartbeat_;

        vector<double> comm_received_;//The size of this vector is robot_number, we don't need to know
        vector<double> comm_received_timestamp_;//the vector to log the timestamp.
    }; 
    
    typedef micros_mars_task_alloc::MotivationalBehavior<std_msgs::Bool> MotivationalBehaviorTest;//TODO: to test the module, the module will be made as a '.h' file in future work.
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::MotivationalBehaviorTest, nodelet::Nodelet)
