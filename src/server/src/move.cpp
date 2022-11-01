#include<ros/ros.h>
#include<std_srvs/Empty.h>
#include<geometry_msgs/Twist.h>
ros::Publisher vel_pub;

bool move_callback(std_srvs::Empty::Request& req , std_srvs::Empty::Response& res)
{
    ROS_INFO("Service in progress");
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0.1;
    velocity.angular.z = 0.1;
    vel_pub.publish(velocity);
    return true;
}

int main(int argc , char** argv )
{
    ros::init(argc,argv,"move_service");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" , 1000);
    ros::ServiceServer move_service = nh.advertiseService("/move_service",move_callback);
    ros::spin();
    return 0 ;
}