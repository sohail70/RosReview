#include<ros/ros.h>
#include<nav_msgs/Odometry.h>


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("%f",msg->pose.pose.position.x);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sub_to_odom");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("odom",1000,odom_callback);
    ros::spin();
}

