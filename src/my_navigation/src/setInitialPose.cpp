#include<ros/ros.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<boost/array.hpp>
int main(int argc , char** argv)
{
    ros::init(argc,argv,"setPose");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);
    
    ros::Duration(5).sleep(); //wait for amcl to be up and running --> TODO: don't hardcode this
    
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = std::string("map");
    pose.pose.pose.position.x = -2.0;
    pose.pose.pose.position.y = -0.5;

    pose.pose.pose.orientation.w = 1.0;
    
    boost::array<double ,36> cov{0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
    pose.pose.covariance = cov;
    ROS_ERROR("SET INITIAL POSE");
    pub.publish(pose);

}