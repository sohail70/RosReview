#include<ros/ros.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<costmap_2d/costmap_2d.h>
#include<nav_core/base_global_planner.h>
#include<PointWrapper.h>

namespace global_planner{

    class Astar: public nav_core::BaseGlobalPlanner{
        public:
            Astar();

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros); 
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        private:
            costmap_2d::Costmap2DROS* costmap_ros;

    };
}