#include<ros/ros.h>
#include<custom_srv_msg/my_custom_srv_msg.h>
#include<geometry_msgs/Twist.h>
ros::Publisher vel_pub;


bool service_callback(custom_srv_msg::my_custom_srv_msg::Request& req ,custom_srv_msg::my_custom_srv_msg::Response& res )
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.1;
    vel.angular.z = 0.1;
    int i = 0;
    ros::Rate loop_rate(1);
    while(i<req.duration)
    {
        vel_pub.publish(vel);
        i++;
        ROS_INFO("Time: %i",i );
        loop_rate.sleep();
    }

    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub.publish(vel);
    res.success = true;
    ROS_INFO("Finished giving service");
    
    return true;


}


int main (int argc ,char** argv)
{
    ros::init(argc,argv,"custom_srv_msg");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" ,1000 );
    ros::ServiceServer custom_service = nh.advertiseService("/custom_service" , service_callback);
    ros::spin();
    return 0;
}