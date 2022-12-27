//https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/   ---> READ IT
//https://answers.ros.org/question/228253/generating-a-2d-costmap-from-a-mapyaml-file/  --> you don't have to read it
#include<iostream>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/PoseStamped.h>
#include<queue>
#include<PointWrapper.h>
#include<nav_msgs/OccupancyGrid.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include<geometry_msgs/Point.h>

namespace DynamicPlanner{

    class LpaStar{
        public:
            LpaStar();
            ~LpaStar();
            void initialize(const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start , const geometry_msgs::PoseStamped& goal , std::vector<geometry_msgs::PoseStamped> plan);
            void computeShortestPath();
            void updateVertex();
            void calculateKey();

            void test();
        private:
            ros::NodeHandle nh;
            ros::Subscriber map_sub;

            std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros;//https://answers.ros.org/question/60026/difference-between-costmap2d-and-occupancygrid-not-clear/
            nav_msgs::OccupancyGrid* map;

            
            // Q(key1 , key2 , index of a node in costmap array)
            std::priority_queue<std::tuple<float, float , int> , std::vector<std::tuple<float, float , int>> ,std::greater<std::tuple<float, float , int>> > q;

            std::unique_ptr<PointWrapper> points1;
            std::unique_ptr<PointWrapper> points2;

    };

};