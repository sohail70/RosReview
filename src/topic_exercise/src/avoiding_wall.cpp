#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>
#include<vector>
#include<tf/tf.h>



class Avoidance{

    public:
        Avoidance()
        {
            scan_received = false;
            odom_received = false;

            vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
            odom_sub = nh.subscribe<nav_msgs::Odometry>("odom",1000,&Avoidance::odom_callback,this);
            laser_sub = nh.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Avoidance::laser_callback,this);
            
        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) // Not Needed!
        {
            tf::Quaternion q(msg->pose.pose.orientation.x ,msg->pose.pose.orientation.y , msg->pose.pose.orientation.z , msg->pose.pose.orientation.w );
            tf::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);
            //ROS_INFO("yaw: %f",yaw);//age jaye %f bezari %d javabe ghalat mide!
            odom_received = true;
        }

        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            size = msg->ranges.size();
            ROS_INFO("Range[0]: %f",msg->ranges.at(0));
            ROS_INFO("Range[pi/2]: %f",msg->ranges.at(1*size/4));
            ROS_INFO("Range[pi]: %f",msg->ranges.at(2*size/4));
            ROS_INFO("Range[3Pi/2]: %f",msg->ranges.at(3*size/4));
            ROS_INFO("-------------------------------------------");
            this->range = msg->ranges;
            scan_received =true;
        }

        void execute()
        {
            while(scan_received==false && odom_received==false)
            {
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }

            ros::Rate loop_rate(1);
            while(ros::ok)
            {
                set_velocity();
                ros::spinOnce();
                loop_rate.sleep();
            }

        }

        void set_velocity()
        {
            if(range[0]>=1) //Move forward
            {
                velocity.linear.x = 0.1;
                velocity.angular.z = 0;
                vel_pub.publish(velocity);
            }
            else if (range[0]<1)
            {
                //turn left
                if(range[3*size/4]<1)
                {
                    velocity.linear.x = 0.03;
                    velocity.angular.z = 0.1;
                    vel_pub.publish(velocity);
                }
                else // turn right
                {
                    velocity.linear.x = 0.05;
                    velocity.angular.z = -0.1;
                    vel_pub.publish(velocity);
                }
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::Subscriber odom_sub;
        ros::Subscriber laser_sub; 
        bool scan_received;
        bool odom_received; 
        std::vector<float> range;
        geometry_msgs::Twist velocity;
        double roll,pitch,yaw;
        int size;
};


int main(int argc , char** argv)
{
    ros::init(argc,argv,"avoidance_node");
    Avoidance avoid;
    avoid.execute();
    return 0;
}
