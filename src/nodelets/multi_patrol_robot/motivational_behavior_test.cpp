/*************************************************************************************************************************
 * Author:         Minglong Li
 * Affiliation:    State Key Laboratory of High Performance Computing (HPCL), College of Computer, National University of Defense Technology
 * Email:            minglong_l@163.com
 * Created on:  Sep. 5th, 2016
 * Description:  In ALLIANCE, the ability for robots to respond to unexpected events, robot failures, and so forth, is 
 *                        provided through the use of motivations. These motivations are designed to allow robot team members
 *                        to perform tasks only as long as they demonstrate their ability to have the desired effect on the world.
 *                        This differs from the commonly used technique for task allocation that begins with breaking down the 
 *                        mission (or part of the mission) into subtasks, and then computing the "optimal" robot-to-task mapping 
 *                        based upon agent skill levels, with little recourse for robot failures after the allocation has occurred.                
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
        MotivationalBehavior(): motivation_(0), sensory_feedback_(1), sensory_feedback_exist_(false), maximum_time_step_(100000), active_time_duration_(0),  one_cycle_(0.1),
                                             delta_fast_(10), delta_slow_(0.01), threshold_(1000), impatience_reset_(1), activity_suppression_(1){}
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
            if(!(nh_.getParam("sensory_feedback_topic", sensory_feedback_topic_)))
            {
                std::cout << "Fail to get the parameter sensory_feedback_topic." << std::endl;
                return;
            }
            if(!(nh_.getParam("intra_robot_comm_topic", intra_robot_comm_topic_)))
            {
                std::cout << "Fail to get the parameter intra_robot_comm_topic." << std::endl;
                return;
            }
            if(!(nh_.getParam("inter_robot_comm_topic", inter_robot_comm_topic_)))
            {
                std::cout << "Fail to get the parameter inter_robot_comm_topic." << std::endl;
                return;
            }
            if(!(nh_.getParam("forward_topic", forward_topic_)))
            {
                std::cout << "Fail to get the parameter forward_topic." << std::endl;
                return;
            }
            //initialise the vector heartbeat_, the row number is robot_number_ and the column number is maximum_time_step_.
            //heartbeat_ is a 2-dimision vector.
            heartbeat_.resize(robot_number_);
            for (int i = 0; i < heartbeat_.size(); i++)
            {
                heartbeat_.resize(maximum_time_step_);
            }
            //Set all the elements in heartbeat_ to zero.
            for (int i = 0; i < heartbeat_.size(); i++)
            {
                for (int j = 0; j < heartbeat_[0].size(); j++)
                {
                    heartbeat_[i][j] = false;
                }
            }
            //initialise the heartbeat_count,  heartbeat_count _  is  a floating point-to-reference, which is used for the ring vector.
            heartbeat_count_.resize(robot_number_);
            for (int i = 0; i < heartbeat_count_.size(); i++)
            {
                heartbeat_count_[i] = 0;
            }
            /*
            //initialise the vector comm_received and comm_received_timestamp_, the default value of the elements in this vector is set to 0.
            for (int i = 0; i < robot_number_; i++)
            {
                comm_received_.push_back(0);
                comm_received_timestamp_.push_back(0);
            }*/
            pub_intra_heartbeat_ = nh_.advertise<std_msgs::Bool>(intra_robot_comm_topic_, 10);
            pub_inter_heartbeat_ = nh_.advertise<std_msgs::Bool>(inter_robot_comm_topic_, 10);
            pub_forward_ = nh_.advertise<std_msgs::Bool>(forward_topic_, 10);
            sub_0_ = nh_.subscribe(sensory_feedback_topic_, 10, &MotivationalBehavior::sensory_feedback_callback, this);//sub_0_ is used to handle the sensory_feedback_ messages.
            sub_1_ = nh_.subscribe(inter_robot_comm_topic_, 10, &MotivationalBehavior::inter_robot_comm_callback, this);//sub_1_ is used to handle the inter-robot communicating messages.
            sub_2_ = nh_.subscribe(intra_robot_comm_topic_, 10, &MotivationalBehavior::intra_robot_comm_callback, this);//sub_2_ is used to hangle the intra-robot communicating messages.
            main_periodic_timer_ = nh_.createTimer(ros::Duration(one_cycle_), &MotivationalBehavior::main_logic_callback, this);//control the main logic.
        }

        void sensory_feedback_callback(const boost::shared_ptr<const VS_MsgType> & msg)
        {
            //Assumption: the sensory_feedback message flow is stable, and the publishing rate is larger than 1 Hz.
            std::cout << "sensory_feedback_callback start!" << std::endl;
            sensory_feedback_exist_ = true;
            sensory_feedback_timestamp_ = ros::Time::now().toSec();//if the sensory_feedback is received, even only once, this flag paramter will be set to true.
            sensory_feedback_ = 1;
        }
        void inter_robot_comm_callback(const micros_mars_task_alloc::HeartbeatConstPtr & msg)
        {
            std::cout << "inter_robot_comm_callback start!" << std::endl;
            if( (msg->robot_ID != this_robot_ID_)&&(msg->behavior_set_ID == this_behavior_set_ID_) )
            {
                heartbeat_[msg->robot_ID][ heartbeat_count_[msg->robot_ID] ] = msg->heartbeat;
                heartbeat_count_[msg->robot_ID]+=1;
                if(heartbeat_count_[msg->robot_ID] == maximum_time_step_)
                    heartbeat_count_[msg->robot_ID] = 0;
            }
        }
        void intra_robot_comm_callback(const micros_mars_task_alloc::HeartbeatConstPtr & msg)
        {
            std::cout << "intra_robot_comm_callback start!" << std::endl;
            //if any other behavior set in this robot is activated, and the robot_ID in the messages must be "this_robot_ID_"
            if((msg->heartbeat==true) && (msg->robot_ID == this_robot_ID_) && (msg->behavior_set_ID != this_behavior_set_ID_))
                activity_suppression_ = 0;
            else//none of other behavior set in this robot is activated.
                activity_suppression_ = 1;
        }
        void impatience_reset_calc()//This function should be put into the main logic function.
        {
            //impatience reset: when this robot hears about another robot performing this behavior set.
            bool flag = false;
            bool flag1, flag2;
            for (int i = 0; i < robot_number_; i ++)
            {
                if (i == this_robot_ID_) continue;
                int current  = heartbeat_count_[i];
                flag1 = true;
                for (int j = 1; j < current; j++)
                {
                    if (heartbeat_[i][j] == true)
                        flag1 = false;
                }
                flag2 = false;
                if (heartbeat_[i][current] == true)
                    flag2 = true;
                if (flag1 && flag2)
                {
                    flag = true;
                    break;
                }
            }
            if (flag == true)
                impatience_reset_ = 1;
            else
                impatience_reset_ = 0;
        }
        void impatience_calc()//This function should be put into the main logic function.
        {
            bool delta_fast_flag = true;
            for (int i = 0; i < robot_number_; i++)//i is robot_ID
            {
                if(i == this_robot_ID_) break;
                int current = heartbeat_count_[i];
                if(heartbeat_[i][current] == true)
                {
                    delta_fast_flag = false;
                    break;
                }
            }
            if(delta_fast_flag)
                impatience_ = delta_fast_;
            else
                impatience_ = delta_slow_;                        
        }
        void acquiescence_calc()
        {
            cout << "acquiescence_calc start" << endl;     
            double phi = 200.0;
            double lamda = 300.0; //behavior set aij of robot ri has been active for more than lamda time.
            if (active_ == 0)
                acquiescence_ = 1;// It means that this beahvior set in this robot is not active.
            else
            {
                 //double current_time = ros::Time::now().toSec();//current_time is a time_stamp.
                 if ( active_time_duration_ > lamda)
                    acquiescence_ = 0;
                else
                {
                    bool flag = false;
                    bool flag1 = false;
                    if (active_time_duration_ > phi)
                        flag1 ==true;
                    bool flag2 = false;
                    for(int i = 0; i < robot_number_; i++)
                    {
                        if (i == this_robot_ID_) continue;
                        int current = heartbeat_count_[i];
                        if(heartbeat_[i][current] == true)
                        {
                            flag2 = true;
                            break;
                        }
                    }
                    flag = flag1 && flag2;
                    if(flag) 
                        acquiescence_ = 0;
                    else
                        acquiescence_ = 1;
                }
            }
        }
        void send_inter_heartbeat(bool heartbeat)
        {
            micros_mars_task_alloc::HeartbeatPtr heartbeat_ptr(new micros_mars_task_alloc::Heartbeat);
            heartbeat_ptr -> robot_ID = this_robot_ID_;
            heartbeat_ptr -> behavior_set_ID = this_behavior_set_ID_;
            heartbeat_ptr -> heartbeat = heartbeat;
            pub_inter_heartbeat_.publish(heartbeat_ptr);
        }
        void send_intra_heartbeat(bool heartbeat)
        {
            micros_mars_task_alloc::HeartbeatPtr heartbeat_ptr(new micros_mars_task_alloc::Heartbeat);
            heartbeat_ptr -> robot_ID = this_robot_ID_;
            heartbeat_ptr -> behavior_set_ID = this_behavior_set_ID_;
            heartbeat_ptr -> heartbeat = heartbeat;
            pub_intra_heartbeat_.publish(heartbeat_ptr);
        }
        void main_logic_callback(const ros::TimerEvent&)
        {
            double update_time;
            update_time = ros::Time::now().toSec();
            std_msgs::BoolPtr bool_ptr(new std_msgs::Bool);
            if ((sensory_feedback_exist_) && (update_time - sensory_feedback_timestamp_ > 1.0))//The time duration for the sensory feedback is 1.0 sensond, the '1.0' can control the real-time efficiency.
            {
                sensory_feedback_ = 0;
            }
            impatience_calc();
            impatience_reset_calc();
            acquiescence_calc();
            motivation_ = (motivation_ + impatience_) * sensory_feedback_ * activity_suppression_ * impatience_reset_ * acquiescence_;
            if(motivation_ > threshold_) 
                motivation_ = threshold_;
            if(motivation_ == threshold_)
             {
                active_ = true;
                active_time_duration_ += one_cycle_;
                bool_ptr -> data = true;
                send_intra_heartbeat(true);
                send_inter_heartbeat(true);
                pub_forward_.publish(bool_ptr);
             } 
             else
             {
                active_ = false;
                active_time_duration_ = 0;
                bool_ptr -> data  = false;
                send_inter_heartbeat(false);
                send_inter_heartbeat(false);
                pub_forward_.publish(bool_ptr);
             }
        }
    private:    
        ros::NodeHandle nh_;
        ros::Timer main_periodic_timer_;
        ros::Subscriber sub_0_;
        ros::Subscriber sub_1_;
        ros::Subscriber sub_2_;
        ros::Publisher pub_intra_heartbeat_;//publish heartbeat messages in this robot, which is used for the control of activity_suppression.
        ros::Publisher pub_inter_heartbeat_;//publish heartbeat messages among team robots, which is used for the control of comm_received.
        ros::Publisher pub_forward_;
        
        int this_robot_ID_;//the ID of this robot, to be set by the paramters in the launch file.
        int this_behavior_set_ID_;//the ID of this behavior set in this robot, to be set by the parameters in the launch file.
        int behavior_set_number_;//the total number of the behavior sets, which is set by the local parameters
        int robot_number_;//the total number of the robots, which is set by the local parameters 
        int maximum_time_step_;

        std::string sensory_feedback_topic_;
        std::string inter_robot_comm_topic_;
        std::string intra_robot_comm_topic_;
        std::string forward_topic_;
    
        //Some parameters used to calculate the motivation, TODO: these may be used as local parameters.
        double motivation_;
        double impatience_;
        double acquiescence_;
        double sensory_feedback_;//TODO: the default value of sensory feedback is 1.
        double sensory_feedback_timestamp_;
        
        bool sensory_feedback_exist_;//Not all motivational behavior exist the sensory feedback item, the default value of this parameter is false, which means it does not exist.
        double activity_suppression_;//if any other behavior set in this robot is activated, the motivation of this behavior set is set to 0. 
        double impatience_reset_;
        
        bool active_;
        double active_time_duration_;
        double one_cycle_;
        
        double delta_slow_;//when another robot is doing this work, the increasing velocity of impatience in this motivational behavior is delta_slow_
        double delta_fast_;//when none of the robots is doing this work, the increasing velocity of impatience in this motivational behavior is delta_fast_
        double threshold_;
        
        vector<int> heartbeat_count_;
        vector< vector<bool> > heartbeat_;

        //comm_received was omitted in this version.
        //vector<double> comm_received_;//The size of this vector is robot_number, we don't need to know
        //vector<double> comm_received_timestamp_;//the vector to log the timestamp.
    }; 
    typedef micros_mars_task_alloc::MotivationalBehavior<std_msgs::Bool> MotivationalBehaviorTest;//TODO: to test the module, the module will be made as a '.h' file in future work.
}//namespace micros_mars_task_alloc
PLUGINLIB_EXPORT_CLASS(micros_mars_task_alloc::MotivationalBehaviorTest, nodelet::Nodelet)
