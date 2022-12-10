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

    enum class status{UNCHECKED , OPEN , CLOSE};
    
    struct Vertex{
        int status;
        int cost;
        float g_value;
        unsigned int cell_x;
        unsigned int cell_y;
        Vertex* parent;
        std::vector<Vertex> neighbors;
        bool operator==(const Vertex& other) const //const ro nazari static_assert error mide // dar kul in operator vase set be in mani hast ke unique beshe set va vase unorderd_multiset be in mani hast ke onae ke g_value yeksan daran adjacent ham bashan vaghti ke roshon iterator mizani
        {
            return g_value == other.g_value;
        }

        bool operator!=(const Vertex& other) const
        {
            return (cell_x != other.cell_x && cell_y != other.cell_y);
        }
    };

    struct Order{
        bool operator()(Vertex a , Vertex b)
        {
            return a.g_value > b.g_value;
        }
    };

    struct MyHash{
        std::size_t operator()(Vertex v) const{
            std::hash<int> hashVal;
            return hashVal(v.g_value);
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
            void neighborsIntoList(Vertex current_node);
            bool boundaryCheck(int cell_x , int cell_y);
            bool statusCheck(int status);

            void validNeighbors(Vertex current_node , std::vector<int>& indices);
        private:
            std::string name;
            costmap_2d::Costmap2DROS* costmap_ros;//in va paeeni ye functionality daran faghat paeeni ke man khodam neveshtamesh pose ro ham mide vali be nazar niazi nabode ke nanveshtan chun dar global planner robot ke harkat nemikune ke modhem bashe ghazie
            // CostmapUtility cUtil; // be inniazi nist chun worldtomap va func e maptoworld dar costmap2dROS inkaro baramoon mikune
            std::priority_queue<Vertex , std::vector<Vertex> , Order> open_list;
            std::unordered_multiset<Vertex,MyHash> closed_list; 
            std::unordered_multiset<Vertex,MyHash> visited_list; 
            PointWrapper* points;


            std::vector<double> distance;
            std::vector<unsigned int> parent;

            std::vector<std::pair<double, Vertex*>> g_value_parent_pair;
            
    };


};

#endif