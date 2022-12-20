#include<iostream>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/PoseStamped.h>

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
    };

};