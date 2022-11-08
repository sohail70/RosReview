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
    goal_.goal = 1;
    client.sendGoal(goal_,&doneCB , &activeCB , &feedbackCB);


    actionlib::SimpleClientGoalState result_state = client.getState();
    ros::Rate loop_rate(2);
    while(result_state == actionlib::SimpleClientGoalState::ACTIVE || result_state== actionlib::SimpleClientGoalState::PENDING)
    {
        ROS_INFO("Doing stuff while waiting for the action server to give a result..");
        loop_rate.sleep();
        result_state = client.getState();
    }
    return 0;

}