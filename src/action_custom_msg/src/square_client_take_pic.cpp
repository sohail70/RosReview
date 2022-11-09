#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/TestAction.h>

#include<action_custom_msg/CustomActionAction.h>

void doneCB(const actionlib::SimpleClientGoalState& state , const actionlib::TestResultConstPtr& result)
{
    ROS_INFO("The action has been completed");
    // ros::shutdown();
}

void activeCB()
{
    ROS_INFO("Goal is active");
}

void feedbackCB(const actionlib::TestFeedbackConstPtr& feedback)
{
    ROS_INFO("[Feedback]: %d received", feedback->feedback);

}

////////////////////////


void doneCB2(const actionlib::SimpleClientGoalState& state , const action_custom_msg::CustomActionResultConstPtr& result)
{
    ROS_INFO("The take pic action has been completed");
}

void activeCB2()
{
    ROS_INFO("Goal 2 is active");
}

void feedbackCB2(const action_custom_msg::CustomActionFeedbackConstPtr& feedback)
{
    ROS_INFO("feedback received");
}






///////////////////////

int main(int argc , char** argv)
{
    ros::init(argc,argv,"square_action_client");
    actionlib::SimpleActionClient<actionlib::TestAction> client("soheil_action" ,  true);
    ROS_INFO("WAITING FOR SERVER 1");
    client.waitForServer(); 
    ////////////////
    actionlib::SimpleActionClient<action_custom_msg::CustomActionAction> client2("take_pic_action",true);
    ROS_INFO("WAITING FOR SERVER 2");
    client2.waitForServer();
    ////////////////

    actionlib::TestGoal goal_;
    goal_.goal = 1;
    client.sendGoal(goal_,&doneCB , &activeCB , &feedbackCB);
    //////////////////
    actionlib::SimpleClientGoalState result_state = client.getState();

    ////////////////
    action_custom_msg::CustomActionGoal goal2_;
    goal2_.seconds = 10;

    ros::Rate loop_rate(1);
    while(result_state == actionlib::SimpleClientGoalState::ACTIVE || result_state== actionlib::SimpleClientGoalState::PENDING)
    {
        ROS_INFO("Doing stuff while waiting for the action server to give a result..");
        actionlib::SimpleClientGoalState result_state_2 = client2.getState();
        if(result_state_2 != actionlib::SimpleClientGoalState::ACTIVE)
        {
            client2.sendGoal(goal2_,&doneCB2,&activeCB2,&feedbackCB2);
        }
        loop_rate.sleep();
        result_state = client.getState();
    }
    return 0;

}