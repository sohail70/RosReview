#include<ros/ros.h>
#include<geometry_msgs/Twist.h>


int main(int argc , char** argv)
{
    ros::init(argc,argv,"Move");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate loop_rate(2);

    geometry_msgs::Twist vel;
    vel.linear.x = 0.1;
    vel.angular.z = 0.01;

    while(ros::ok)
    {
        pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 