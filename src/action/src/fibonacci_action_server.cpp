#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<actionlib_tutorials/FibonacciAction.h>
#include<functional>

class FiboAction
{
    public:
        FiboAction(std::string name): action_name(name) ,as_(nh_,name,std::bind(&FiboAction::executeCB , this,std::placeholders::_1) , false)
        {
            as_.start();
        }
        void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr & goal)
        {
            ros::Rate loop_rate(1);
            bool success = true;
            feedback_.sequence.clear();
            feedback_.sequence.push_back(0);
            feedback_.sequence.push_back(1);
            ROS_INFO("%s: Executing, creaing fobinacci sequence of order %i with seeds %i ,%i",action_name.c_str() , goal->order , feedback_.sequence[0] , feedback_.sequence[1]);

            for(int i = 0 ; i<=goal->order ; i++)
            {
                if(as_.isPreemptRequested() || !ros::ok)
                {
                    ROS_INFO("%s: preempted", action_name.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }
                feedback_.sequence.push_back(feedback_.sequence[i]+feedback_.sequence[i-1]);
                as_.publishFeedback(feedback_);
                loop_rate.sleep();
            }

            if(success)
            {
                result_.sequence = feedback_.sequence;
                ROS_INFO("%s: succeeded", action_name.c_str());
                as_.setSucceeded(result_);
            }



        }

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;
        std::string action_name;
        actionlib_tutorials::FibonacciFeedback feedback_;
        actionlib_tutorials::FibonacciResult result_;
         


};

int main(int argc , char** argv)
{
    ros::init(argc,argv , "fibo_action_server");
    FiboAction fibonacci("fibonacci");
    ros::spin();
    return 0 ;

    
}