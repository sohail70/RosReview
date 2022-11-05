#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/TestAction.h>



void doneCB(const actionlib::SimpleClientGoalState& state , const actionlib::TestResultConstPtr& result)
{
    ROS_INFO("The action has been completed");
    ros::shutdown();
}

void activeCB()
{
    ROS_INFO("Goal is active");
}

void feedbackCB(const actionlib::TestFeedbackConstPtr& feedback)
{
    ROS_INFO("[Feedback]: %d received", feedback->feedback);

}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"square_action_client");
    actionlib::SimpleActionClient<actionlib::TestAction> client("soheil_action" ,  true);
    client.waitForServer(); 

    actionlib::TestGoal goal_;
    goal_.goal = 2;
    client.sendGoal(goal_,&doneCB , &activeCB , &feedbackCB);

    client.waitForResult();
    return 0;

}