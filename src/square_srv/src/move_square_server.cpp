#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<square_srv/square.h>
#define PI 3.14
class Service{
    public:
        Service()
        {
            vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
            server = nh.advertiseService("square_service" , &Service::service_callback,this);
            
            vel_lin.linear.x = 0.2;
            vel_lin.angular.z = 0;

            vel_ang.angular.z = 0.2;

            vel_stop.linear.x = 0;
        }

        bool service_callback(square_srv::square::Request& req , square_srv::square::Response& res)
        {
            int steps;
            double time_needed;
            for (int rep = 0 ; rep < req.repetition ; rep++)
            {
                steps = 0;
                while (steps<4)
                {
                    time_needed = req.side / vel_lin.linear.x;
                    vel_pub.publish(vel_lin);
                    ros::Duration(time_needed).sleep();
                    vel_pub.publish(vel_stop);
                    time_needed = (PI/2) / vel_ang.angular.z;
                    vel_pub.publish(vel_ang);
                    ros::Duration(time_needed).sleep();

                    vel_pub.publish(vel_stop);
                    steps++;
                }
                
            }
            ROS_INFO("Finished moving in square");
            res.success = true;
            return true;
        }


    private:
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::ServiceServer server;
        geometry_msgs::Twist vel_lin;
        geometry_msgs::Twist vel_ang;
        geometry_msgs::Twist vel_stop;

};  




int main(int argc , char** argv)
{
    ros::init(argc,argv,"square_server_node");
    Service serv;
    ros::spin();
    return 0;
}