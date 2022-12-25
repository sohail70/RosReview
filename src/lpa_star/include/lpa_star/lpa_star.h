#include<iostream>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/PoseStamped.h>
#include<queue>
#include<PointWrapper.h>



namespace DynamicPlanner{

    class LpaStar{
        public:
            LpaStar();
            ~LpaStar();
            void initialize(const costmap_2d::Costmap2D& costmap);
            bool makePlan(const geometry_msgs::PoseStamped& start , const geometry_msgs::PoseStamped& goal , std::vector<geometry_msgs::PoseStamped> plan);
            void updatePlan(std::vector<geometry_msgs::PoseStamped>& plan);
            
        private:
            costmap_2d::Costmap2D* costmap;
            // Q(key1 , key2 , index of a node in costmap array)
            std::priority_queue<std::tuple<float, float , int> , std::vector<std::tuple<float, float , int>> ,std::greater<std::tuple<float, float , int>> > q;

            std::unique_ptr<PointWrapper> points;
            std::unique_ptr<PointWrapper> points2;

    };

};