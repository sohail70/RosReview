#ifndef DIJKSTRA_H
#define DIJKSTRA_H


#include<ros/ros.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<costmap_2d/costmap_2d.h>
#include<nav_core/base_global_planner.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>

namespace global_planner{
    class Dijkstra: public nav_core::BaseGlobalPlanner
    {
        public:

            Dijkstra();
            Dijkstra(std::string name_ , costmap_2d::Costmap2DROS* costmap_ros);


            void initialize(std::string name_, costmap_2d::Costmap2DROS* costmap_ros) override;

            bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

            void show();

        private:
            std::string name;
            costmap_2d::Costmap2DROS* costmap;
            
    };


};

#endif