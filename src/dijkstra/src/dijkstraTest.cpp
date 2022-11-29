// https://index.ros.org/p/class_loader/

#include<ros/ros.h>
#include <class_loader/class_loader.hpp>
#include<nav_core/base_global_planner.h>
#include<Dijkstra.h>
#include<vector>
#include<boost/shared_ptr.hpp>

#include<aLib.h>
int main(int argc , char** argv)
{
    ros::init(argc , argv , "Test");

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




    /* Just a test to use a shared library inside my cpp file */
    // aLib a;
    // std::cout<<a.sum(2,3)<<std::endl;
    

}