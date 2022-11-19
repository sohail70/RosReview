#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<vector>

void doneCB(const actionlib::SimpleClientGoalState& state , const move_base_msgs::MoveBaseResultConstPtr& result){
    ROS_INFO("DONE");
}


void activeCB(){
    ROS_INFO("ACTIVE");

}


void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
    ROS_INFO("FEEDBACK");
}



int main(int argc , char** argv)
{
    ros::init(argc, argv , "goal_client");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("/move_base" , true);

    client.waitForServer();
    std::vector<move_base_msgs::MoveBaseGoal> goals;

    goals.resize(3);

    goals[0].target_pose.pose.position.x = 2.0;
    goals[0].target_pose.pose.position.y = -0.7;
    goals[0].target_pose.pose.orientation.w = 1.0;
    goals[0].target_pose.header.frame_id = "map";


    goals[1].target_pose.pose.position.x = 2.0;
    goals[1].target_pose.pose.position.y = 0.7;
    goals[1].target_pose.pose.orientation.w = 1.0;
    goals[1].target_pose.header.frame_id = "map";


    goals[2].target_pose.pose.position.x = -2.0;
    goals[2].target_pose.pose.position.y = -0.7;
    goals[2].target_pose.pose.orientation.w = 1.0;
    goals[2].target_pose.header.frame_id = "map";

    move_base_msgs::MoveBaseGoal current_goal;
    
    int i = 0;
    while(true)
    {
        current_goal = goals[i];
        client.sendGoal(current_goal,&doneCB , &activeCB , &feedbackCB);
        client.waitForResult();    
        ROS_INFO("Reached");
        if(i<2)
        {
            i++;
        }
        else{
            i = 0;
        }
    }

    
}