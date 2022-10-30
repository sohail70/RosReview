#include<ros/ros.h>

int main(int argc , char** argv)
{
    ros::init(argc,argv,"HelloWorld");
    ros::NodeHandle nh;
    
    ros::Rate loop_rate(2); //2hz
    while(ros::ok)
    {
        ROS_INFO("HELLO WORLD ");
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    return 0;
}