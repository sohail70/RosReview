#include<Dijkstra.h>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Point.h>
#include<set>
// for ros info https://answers.ros.org/question/233107/why-doesnt-ros_info-allow-calling-stdqueuesize-inside-as-a-param/

std::set<int> values;

ros::Publisher marker_pub ;
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    // ROS_INFO("MAP SIZE: %zd",data->data.size()); //256*128 = tedade pixel e to naghshe --> map size
    visualization_msgs::Marker points;
    points.header.frame_id = std::string("map");
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.2;
    points.scale.y = 0.2;
 
    geometry_msgs::Point p;
    p.x = -3;
    p.y = -3;
    p.z = 0;
    
    points.points.push_back(p);
    marker_pub.publish(points);

    for(auto& g : data->data)
    {
        values.insert(g);
    }

    for(auto v: values)
    {
        ROS_INFO("NUMS: %i",v);
    }
}


int main(int argc , char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/marker" , 1);
    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",1,&costmap_callback);
    ros::spin();

    return 0;
}