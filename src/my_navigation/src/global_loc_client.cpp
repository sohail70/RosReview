#include<ros/ros.h>

#include<std_srvs/Empty.h>

int main(int argc , char** argv)
{
    ros::init(argc,argv , "client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/global_localization");
    client.waitForExistence();
    
    std_srvs::Empty srv;

    if(client.call(srv))
    {
        ROS_INFO("Done calling the service");
    }
    else
    { 
        ROS_INFO("Failed calling the service");
    }


}