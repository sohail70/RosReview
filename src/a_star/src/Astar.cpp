#include "Astar.h"
#include<pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::Astar , nav_core::BaseGlobalPlanner)

namespace global_planner{
    Astar::Astar(){};


    void Astar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {

    }

    
    bool Astar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ROS_ERROR("ASTAR STARTS");

        return true;
    }
}