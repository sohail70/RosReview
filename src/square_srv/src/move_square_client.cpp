#include<ros/ros.h>

#include<square_srv/square.h>



int main(int argc , char** argv)
{
    ros::init(argc,argv, "square_client_node");
    ros::NodeHandle nh;

    ros::ServiceClient cli = nh.serviceClient<square_srv::square>("/square_service");

    square_srv::square srv;
    srv.request.side = 1;
    srv.request.repetition = 1;
    if(cli.call(srv))
    {
        ROS_INFO("Done calling the service");
    }
    else
    {
        ROS_INFO("Failed calling the service");
    }

    srv.request.side = 2;
    srv.request.repetition = 1;
    if(cli.call(srv))
    {
        ROS_INFO("Done calling the service");
    }
    else
    {
        ROS_INFO("Failed calling the service");
    }




}