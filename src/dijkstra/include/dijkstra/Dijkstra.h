/*
Open list
close list
visited 
find neighbors
start goal ---> conversion from poistion to cell indices and vice verca functions  --> follow single responsibility rule ---> mathematical stuff like the conversion should be outside --> maybe in a mapWrapper class to add some functionality to the cost map
expand

remember to create a utility class for visualization


*/

/*
    getCost function returns these numbers

    255 ---> unknown
    Pink Color ---> less than 254(i think)
    Red color ---> less than Pink
    Blue ---> less than red
    Pale blue ---> less than blue
    White ---> 0 ---> white space
    
    

    ba dastor paeen cost haro check kun ---> khoob mishod age probability ro migereftim yani -1,[0,100] ke dar costmap migereftim
    ROS_ERROR("COOOOOST %d" , costmap_ros->getCostmap()->getCost(25,20));
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
#include<unordered_set>
#include<algorithm>
#include<numeric>


#include"PointWrapper.h"
#include"CostmapUtility.h"

namespace global_planner{

    class Dijkstra: public nav_core::BaseGlobalPlanner
    {
        public:

            Dijkstra();
            void initialize(std::string name_, costmap_2d::Costmap2DROS* costmap_ros) override;
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;
            void validNeighbors(unsigned int current_node_index , std::vector<unsigned int>& indices);

        private:
            std::string name;
            costmap_2d::Costmap2DROS* costmap_ros;
            std::priority_queue<std::pair<unsigned int,unsigned int> , std::vector<std::pair<unsigned int, unsigned int>> , std::greater<std::pair<unsigned int, unsigned int>>> open_list;
            std::vector<unsigned int> g_value;            




            std::unique_ptr<PointWrapper> points;
            std::unique_ptr<PointWrapper> points2;

            std::vector<unsigned int> parent;
            std::vector<unsigned int> visited;
            
    };


};

#endif