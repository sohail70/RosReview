#include<ros/ros.h>
#include<nav_msgs/GetMap.h>



int main(int argc , char** argv)
{
    ros::init(argc,argv,"map_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/static_map");

    client.waitForExistence(); // ino nazari naghshe ro nemigire!
    
    nav_msgs::GetMap srv;
    
    if(client.call(srv))
    {
        ROS_INFO("I got the map");
    }
    else{
        ROS_INFO("I haven't gotten the map");
    }
    
}