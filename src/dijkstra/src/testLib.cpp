#include<Dijkstra.h>

#include<nav_msgs/OccupancyGrid.h>

// for ros info https://answers.ros.org/question/233107/why-doesnt-ros_info-allow-calling-stdqueuesize-inside-as-a-param/
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    ROS_INFO("MAP SIZE: %zd",data->data.size());
}


int main(int argc , char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",1,&costmap_callback);

    ros::spin();

    return 0;
}