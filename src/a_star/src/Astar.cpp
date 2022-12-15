//http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7  ---> Guidelines for A star ---> THIS IS VERY IMPORTANT FOR CHOOSING A HEURISTIC
/*
    some important notes from the website above:
    Use the distance heuristic that matches the allowed movement:
        On a square grid that allows 4 directions of movement, use Manhattan distance (L1).
        On a square grid that allows 8 directions of movement, use Diagonal distance (L∞).
        On a square grid that allows any direction of movement, you might or might not want Euclidean distance (L2). If A* is finding paths on the grid but you are allowing movement not on the grid, you may want to consider other representations of the map.
        On a hexagon grid that allows 6 directions of movement, use Manhattan distance adapted to hexagonal grids.

*/



#include "Astar.h"
#include<pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::Astar , nav_core::BaseGlobalPlanner)

namespace global_planner{
    Astar::Astar(){};


    void Astar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        this->costmap_ros = costmap_ros;
        std_msgs::ColorRGBA color; color.a =1 ; color.g = 1;
        std_msgs::ColorRGBA color1; color1.a = 1; color1.r = 1;
        points = std::make_unique<PointWrapper>(color , "map");
        points2 = std::make_unique<PointWrapper>(color1 , "map");
        
    }

    
    bool Astar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ROS_INFO("ASTAR STARTS");

        points->initialize(); // Clear the previous points
        points2->initialize();
        points->publish();
        points2->publish();
        geometry_msgs::Point p;



        const unsigned int width = 256 ; // maybe you could use ros parameters to get this 
        const unsigned int height = 128;
        const unsigned int number_of_cells = width*height;
        std::array< float , number_of_cells> f_value; f_value.fill(987654321); 
        std::array< float , number_of_cells> g_value; g_value.fill(987654321);
        std::array<unsigned int , number_of_cells> visited; visited.fill(0);
        std::array<unsigned int , number_of_cells> parent; parent.fill(-1);


        unsigned int start_cell_x , start_cell_y , goal_cell_x , goal_cell_y;
        costmap_ros->getCostmap()->worldToMap(start.pose.position.x , start.pose.position.y ,start_cell_x , start_cell_y);
        unsigned int start_index = costmap_ros->getCostmap()->getIndex(start_cell_x , start_cell_y);

        costmap_ros->getCostmap()->worldToMap(goal.pose.position.x , goal.pose.position.y , goal_cell_x , goal_cell_y);
        unsigned int goal_index = costmap_ros->getCostmap()->getIndex(goal_cell_x , goal_cell_y);


        f_value.at(start_index) = 0 + hValue(start_index , goal_index); // f_value = g_value + heuristic
        g_value.at(start_index) = 0;
        parent.at(start_index) = -2;

        open_list = std::priority_queue<std::pair<float , unsigned int> , std::vector<std::pair<float  ,unsigned int>> , std::greater<std::pair<float , unsigned int>>>();
        open_list.push(std::pair<float , unsigned int>(f_value.at(start_index) , start_index));

        while(!open_list.empty())
        {
            auto top = open_list.top(); //**
            open_list.pop();
            unsigned int v = top.second ;
            float d = top.first;
        

            if (v == goal_index)
            {
                break;
            } 

            if(d <= f_value[v] && visited[v]==false) //**
            {
                std::vector<unsigned int>  valid_neighbors_indices;        
                validNeighbors(v, valid_neighbors_indices);
                for (auto& n_index : valid_neighbors_indices)
                {
                    unsigned int v2 = n_index , cost = 1;
                    if(g_value[v2] > g_value[v] + cost)
                    {
                        g_value[v2] = g_value[v] + cost;
                        float h_value = hValue(v2 , goal_index);
                        unsigned int v2_cell_x , v2_cell_y; costmap_ros->getCostmap()->indexToCells(v2 , v2_cell_x , v2_cell_y); //**
                        tieBreaker(h_value, v2_cell_x , v2_cell_y , start_cell_x , start_cell_y , goal_cell_x , goal_cell_y); //**
                        open_list.push(std::pair<float , unsigned int>(g_value[v2] +h_value  , v2)); // **
                        parent[v2] = v;
                    }
                }
            }
            visited[v] = 1;

            unsigned int cell_x , cell_y;

            costmap_ros->getCostmap()->indexToCells(v , cell_x , cell_y);
            costmap_ros->getCostmap()->mapToWorld(cell_x ,cell_y , p.x , p.y);
            points->addPoint(p);
            points->publish();

        }


        /* if you don't want the robot to move you can delete the below codes until return true to just test the A-star algorithm on different goal very fast without moving the robot */
        geometry_msgs::PoseStamped pos;
        pos.pose.orientation.w = 1;
        pos.header.frame_id="map";

        unsigned int current_index = goal_index;
        while(parent[current_index]!=-2)
        {
            unsigned int cell_x , cell_y ;
            costmap_ros->getCostmap()->indexToCells(current_index , cell_x , cell_y);
            costmap_ros->getCostmap()->mapToWorld(cell_x , cell_y , p.x , p.y);
            points2->addPoint(p);
            points2->publish();

            pos.pose.position.x = p.x;
            pos.pose.position.y = p.y;
            plan.push_back(pos);

            current_index = parent[current_index];

            
        }
        plan[0].pose.orientation = goal.pose.orientation; 
        std::reverse(plan.begin() , plan.end()); 

        ROS_INFO("Finish Building route");




        return true;
    }

    float Astar::hValue(unsigned int cell_index , unsigned int goal_index)
    {
        unsigned int cell_x , cell_y , goal_cell_x , goal_cell_y;
        costmap_ros->getCostmap()->indexToCells(cell_index , cell_x , cell_y);
        costmap_ros->getCostmap()->indexToCells(goal_index , goal_cell_x , goal_cell_y);
        float scale = 1;
        float dist = scale*man(cell_x , cell_y , goal_cell_x , goal_cell_y);
        return dist; 
        

        // return man();
    }

    float Astar::euc(int a_cell_x , int a_cell_y , int b_cell_x , int b_cell_y)
    {
        return std::sqrt(std::pow((a_cell_x -b_cell_x) , 2) + std::pow((a_cell_y - b_cell_y),2));
    }

    float Astar::man(int a_cell_x , int a_cell_y , int b_cell_x , int b_cell_y)
    {
        return std::abs(a_cell_x - b_cell_x) + std::abs(a_cell_y - b_cell_y); 
    }

    void Astar::tieBreaker(float& h_value , int cell_x , int cell_y ,int start_cell_x , int start_cell_y, int goal_cell_x , int goal_cell_y)
    {
        int dx1 = cell_x - goal_cell_x;
        int dy1 = cell_y - goal_cell_y;
        int dx2 = start_cell_x - goal_cell_x;
        int dy2 = start_cell_y - goal_cell_y;
        float cross = std::abs(dx1*dy2 - dx2*dy1); //Cross product of start-goal vector and current-goal vector
        h_value += cross*0.001;
    }

    void Astar::validNeighbors(unsigned int current_node_index , std::vector<unsigned int>& neighbors)
    {
        unsigned int current_node_cell_x , current_node_cell_y;
        costmap_ros->getCostmap()->indexToCells(current_node_index, current_node_cell_x , current_node_cell_y);
        
        
        // right
        unsigned int cell_x_right = current_node_cell_x + 1;
        unsigned int  cell_y_right = current_node_cell_y;
        if(costmap_ros->getCostmap()->getCost(cell_x_right , cell_y_right) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_right , cell_y_right);
            neighbors.push_back(index);   
        } 

        // up
        unsigned int cell_x_up = current_node_cell_x ;  
        unsigned int cell_y_up = current_node_cell_y + 1;
        if(costmap_ros->getCostmap()->getCost(cell_x_up , cell_y_up) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_up , cell_y_up);
            neighbors.push_back(index);   
        }  

        // left
        int cell_x_left = current_node_cell_x - 1;
        int cell_y_left = current_node_cell_y;
        if(costmap_ros->getCostmap()->getCost(cell_x_left , cell_y_left) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_left , cell_y_left);
            neighbors.push_back(index);   
        } 

        // down
        int cell_x_down = current_node_cell_x;
        int cell_y_down = current_node_cell_y - 1;
        if(costmap_ros->getCostmap()->getCost(cell_x_down , cell_y_down) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_down , cell_y_down);
            neighbors.push_back(index);   
        } 

    }
}