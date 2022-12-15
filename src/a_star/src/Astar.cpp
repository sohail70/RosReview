#include "Astar.h"
#include<pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::Astar , nav_core::BaseGlobalPlanner)

namespace global_planner{
    Astar::Astar(){};


    void Astar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        this->costmap_ros = costmap_ros;
        std_msgs::ColorRGBA color; color.a =1 ; color.g = 1;
        std_msgs::ColorRGBA color1; color1.a = 1; color1.r = 1;
        points = std::make_unique<PointWrapper>(color , "map");
        points2 = std::make_unique<PointWrapper>(color1 , "map");
        
    }

    
    bool Astar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ROS_ERROR("ASTAR STARTS");

        unsigned int start_index , start_cell_x , start_cell_y ,goal_index ,  goal_cell_x , goal_cell_y;
        costmap_ros->getCostmap()->worldToMap(start.pose.position.x, start.pose.position.y , start_cell_x , start_cell_y );
        costmap_ros->getCostmap()->worldToMap(goal.pose.position.x , goal.pose.position.y , goal_cell_x , goal_cell_y);
        costmap_ros->getCostmap()->getIndex(start_cell_x , start_cell_y);
        costmap_ros->getCostmap()->getIndex(goal_cell_x , goal_cell_y );


        return true;
    }

    float Astar::hValue(unsigned int cell_index , unsigned int goal_index)
    {
        unsigned int cell_x , cell_y , goal_cell_x , goal_cell_y;
        costmap_ros->getCostmap()->indexToCells(cell_index , cell_x , cell_y);
        costmap_ros->getCostmap()->indexToCells(goal_index , goal_cell_x , goal_cell_y);
        return euc(cell_x , cell_y , goal_cell_x , goal_cell_y); 
        // return man();
    }

    float euc(int a_cell_x , int a_cell_y , int b_cell_x , int b_cell_y)
    {
        return std::sqrt(std::pow((a_cell_x -b_cell_x) , 2) + std::pow((a_cell_y - b_cell_y),2));
    }

    float man(int a_cell_x , int a_cell_y , int b_cell_x , int b_cell_y)
    {
        return std::abs(a_cell_x - b_cell_x) + std::abs(a_cell_y - b_cell_y); 
    }

}