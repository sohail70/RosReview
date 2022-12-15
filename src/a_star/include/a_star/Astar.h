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
            
            void validNeighbors(unsigned int current_node_index , std::vector<unsigned int>& neighbors);

            // tieBreaker : to prefer paths that are along the straight line from the starting point to the goal
            // When these vectors don’t line up, the cross product will be larger. The result is that this code will give some slight preference to a path that lies along the straight line path from the start to the goal. When there are no obstacles, A* not only explores less of the map, the path looks very nice as well
            void tieBreaker(float& h_value , int cell_x , int cell_y ,int start_cell_x , int start_cell_y, int goal_cell_x , int goal_cell_y);
        private:
            costmap_2d::Costmap2DROS* costmap_ros;
            // Q(f_value , index)
            std::priority_queue<std::pair<float , unsigned int> , std::vector<std::pair<float , unsigned int>> , std::greater<std::pair<float , unsigned int>>> open_list;

            std::unique_ptr<PointWrapper> points;
            std::unique_ptr<PointWrapper> points2;

    };
}