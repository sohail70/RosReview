#include<ros/ros.h>
#include<nav_msgs/GetPlan.h>


/*
/move_base/make_plan
nav_msgs/GetPlan
*/
int main(int argc , char** argv)
{
    ros::init(argc ,argv , "make_plan_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = std::string("map");
    srv.request.start.pose.position.x = -2.0;
    srv.request.start.pose.position.y = -0.5;
    srv.request.start.pose.orientation.w = 1;

    srv.request.goal.header.frame_id = std::string("map");
    srv.request.goal.pose.position.x = 2.0;
    srv.request.goal.pose.position.y = -0.5;
    srv.request.goal.pose.orientation.w = 1;

    if(client.call(srv))
    {
        ROS_INFO("Called make_plan service");
    }
    else{
        ROS_INFO("Failed to call make_plan service");
    }

}