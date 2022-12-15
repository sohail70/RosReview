#include<ros/ros.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<costmap_2d/costmap_2d.h>
#include<nav_core/base_global_planner.h>
#include<PointWrapper.h>
#include<queue>
#include<array>
#include<cmath>
namespace global_planner{

    class Astar: public nav_core::BaseGlobalPlanner{
        public:
            Astar();

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros); 
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            float hValue(unsigned int cell_index ,  unsigned int goal_index);
            float euc(int a_cell_x , int a_cell_y, int b_cell_x , int b_cell_y);

            float man(int a_cell_x , int a_cell_y, int b_cell_x , int b_cell_y);
            
        private:
            costmap_2d::Costmap2DROS* costmap_ros;
            // Q(g_value , index)
            std::priority_queue<std::pair<unsigned int , unsigned int> , std::vector<std::pair<unsigned int , unsigned int>> , std::greater<std::pair<unsigned int , unsigned int>>> open_list;
            

            std::unique_ptr<PointWrapper> points;
            std::unique_ptr<PointWrapper> points2;

    };
}