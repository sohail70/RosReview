#include<Dijkstra.h>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<set>
#include<vector>
#include<cmath>
// for ros info https://answers.ros.org/question/233107/why-doesnt-ros_info-allow-calling-stdqueuesize-inside-as-a-param/

std::set<int> values;
nav_msgs::OccupancyGrid costmap;
std::vector<int> obstacles_high_value;
std::vector<int> obstacles_low_value;
std::vector<int> free_space;
std::vector<int> unknown_space;


nav_msgs::Odometry odom;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    // ROS_INFO("MAP SIZE: %zd",data->data.size()); //256*128 = tedade pixel e to naghshe --> map size
   
    
    for (unsigned int i = 0 ; i< data->data.size() ; i++)
    {
        if (data->data.at(i) == 100)
        {
            obstacles_high_value.push_back(i);
        }
        else if (data->data.at(i) < 100 && data->data.at(i)>0)
        {
            obstacles_low_value.push_back(i);
        }
        else if(data->data.at(i) == 0)
        {
            free_space.push_back(i);
        }
        else
        {
            unknown_space.push_back(i);
        }
    }

    for(auto index: obstacles_high_value)
    {
        ROS_INFO("NUMS: %i",index);
    }


    costmap = *data;
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
    odom = *data;
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
{
    amcl_pose = *data;
}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::Publisher pub_obs_high  = nh.advertise<visualization_msgs::Marker>("/marker_high" , 1000);
    ros::Publisher pub_obs_low  = nh.advertise<visualization_msgs::Marker>("/marker_low" , 1000);

    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",1,&costmap_callback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",1,&odom_callback );
    ros::Subscriber amcl_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1,&amcl_callback );
    
    
    
    

    ros::Rate loop_rate(1);
    
    while(ros::ok())
    {
        visualization_msgs::Marker points , points2;
        points.header.frame_id  = points2.header.frame_id = std::string("map");
        points.header.stamp = points2.header.stamp = ros::Time::now();
        points.action = points2.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = points2.pose.orientation.w = 1.0;
        points.id  = points2.id = 0;
        points.type = points2.type = visualization_msgs::Marker::POINTS;

        points.scale.x = points2.scale.x  = 0.05;
        points.scale.y = points2.scale.y = 0.05;
    
        geometry_msgs::Point odom_point , amcl , corner_point , sample_point;
        odom_point.x = odom.pose.pose.position.x;
        odom_point.y = odom.pose.pose.position.y;
        odom_point.z = 0;

        amcl.x = amcl_pose.pose.pose.position.x;
        amcl.y = amcl_pose.pose.pose.position.y;

        //res * cell_x = pose_x
        //res * cell_y = pose_y
        corner_point.x = costmap.info.origin.position.x + costmap.info.resolution * 256;
        corner_point.y = costmap.info.origin.position.y;
        
        // //30995 index
        // int cell_y = 30995 / 256;
        // int cell_x = fmod(30995,256);
        // sample_point.x = costmap.info.origin.position.x + costmap.info.resolution * cell_x;
        // sample_point.y = costmap.info.origin.position.y + costmap.info.resolution * cell_y;
        
        //*********************************

        points.color.r = 1.0f;
        points.color.a = 1.0;

        points2.color.b = 1.0f;
        points2.color.a = 1.0;
        
        points.points.push_back(odom_point);
        points.points.push_back(amcl);
        points.points.push_back(corner_point);


        //**********************************
        
        for (auto i : obstacles_high_value)
        {
            geometry_msgs::Point p;
            int cell_y = i / 256;
            int cell_x = fmod(i,256);
            p.x = costmap.info.origin.position.x + costmap.info.resolution * cell_x + (costmap.info.resolution /2);
            p.y = costmap.info.origin.position.y + costmap.info.resolution * cell_y + (costmap.info.resolution /2);
            points.points.push_back(p);
        }

        for (auto i : obstacles_low_value)
        {
            geometry_msgs::Point p;
            int cell_y = i / 256;
            int cell_x = fmod(i,256);
            p.x = costmap.info.origin.position.x + costmap.info.resolution * cell_x + (costmap.info.resolution /2);
            p.y = costmap.info.origin.position.y + costmap.info.resolution * cell_y + (costmap.info.resolution /2);
            points2.points.push_back(p);
        }


        pub_obs_high.publish(points);
        pub_obs_low.publish(points2);


        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    return 0;
}