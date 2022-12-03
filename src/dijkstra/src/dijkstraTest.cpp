// https://index.ros.org/p/class_loader/

#include<ros/ros.h>
#include <class_loader/class_loader.hpp>
#include<nav_core/base_global_planner.h>
#include<vector>
#include<boost/shared_ptr.hpp>

#include"CostmapUtility.h"
#include"Dijkstra.h"
#include"PointWrapper.h"


int main(int argc , char** argv)
{
    ros::init(argc , argv , "Test");
    ros::NodeHandle nh;
/********************************************************************************************************/
    class_loader::ClassLoader loader("libmyDijkstra.so");
    std::vector<std::string> classes = loader.getAvailableClasses<nav_core::BaseGlobalPlanner>();
    std::vector<boost::shared_ptr<nav_core::BaseGlobalPlanner>> plugins;
    
    for(unsigned int c = 0 ; c <classes.size() ; c++)
    {
        std::cout<<classes[c]<<"\n";
        plugins.push_back(loader.createInstance<nav_core::BaseGlobalPlanner>(classes[c]));
    }
    
    auto base_object = plugins[0];

    auto dijkstra_object = dynamic_cast<global_planner::Dijkstra*>(base_object.get());
    dijkstra_object->show();




/********************************************************************************************************/

    CostmapUtility costmap_util;
    geometry_msgs::Point p ;
    p.x = 22.6;
    p.y = -3;
    auto index =  costmap_util.pose_to_cell(p);
    ROS_INFO("index: %i", index);


/*********************************************/
    std_msgs::ColorRGBA color; 
    color.a = 1.0;
    color.r = 1.0;

    std_msgs::ColorRGBA color2;
    color2.a = 1.0;
    color2.g = 1.0;
    
    PointWrapper marker(color , std::string("map"));
    marker.addPoint(p);
    PointWrapper marker2(color2 , std::string("map"));

    geometry_msgs::Point p2;
    p2.x  = 5;
    p2.y = 0;
    marker2.addPoint(p2);
    int i = 0;
    ros::Rate loop_rate(0.5);
    while (ros::ok)
    {
        marker.deletePoint(0); 
        p.x = -3 + 0.07*i;
        p.y = -3 + 0.07*i;
        marker.addPoint(p);
        marker.publish();
        marker2.publish();
        ros::spinOnce();
        i++;
        loop_rate.sleep(); 
    }
    

}