#include<ros/ros.h>
#include<std_srvs/Empty.h>

bool my_callback(std_srvs::Empty::Request &req , std_srvs::Empty::Response &res)
{
    // resp.some_variable  = req.some_variable + req.other_variable
    ROS_INFO("my_callback has been called");
    return true;
}


int main(int argc , char** argv)
{
    ros::init(argc,argv,"server");
    ros::NodeHandle nh;
    ros::ServiceServer my_service = nh.advertiseService("/my_service",my_callback);
    ros::spin();
    return 0;
}