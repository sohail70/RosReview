/*
Open list
close list
visited 
find neighbors
start goal ---> conversion from poistion to cell indices and vice verca functions  --> follow single responsibility rule ---> mathematical stuff like the conversion should be outside --> maybe in a mapWrapper class to add some functionality to the cost map
expand

remember to create a utility class for visualization


*/



#ifndef DIJKSTRA_H
#define DIJKSTRA_H


#include<ros/ros.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<costmap_2d/costmap_2d.h>
#include<nav_core/base_global_planner.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include<queue>


#include"PointWrapper.h"
#include"CostmapUtility.h"

namespace global_planner{

    struct Vertex{
        float g_value;
        int cell_x;
        int cell_y;
    };

    struct Order{
        bool operator()(Vertex a , Vertex b)
        {
            return a.g_value > b.g_value;
        }
    };


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
            costmap_2d::Costmap2DROS* costmap_ros;//in va paeeni ye functionality daran faghat paeeni ke man khodam neveshtamesh pose ro ham mide vali be nazar niazi nabode ke nanveshtan chun dar global planner robot ke harkat nemikune ke modhem bashe ghazie
            CostmapUtility cUtil; // felan bashe age niazi nabood pakesh kun
            std::priority_queue<Vertex , std::vector<Vertex> , Order> open_list;
            
            
    };


};

#endif