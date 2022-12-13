// follow the rules here http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
// Note : Read https://stackoverflow.com/questions/649640/how-to-do-an-efficient-priority-update-in-stl-priority-queue
// baraye use kardan az func haye costmap2d_ ros  ----> ino dar cmake yadet nare be library myDijkstra ---> /opt/ros/noetic/lib/libcostmap_2d.so --> alabte vase func e getname niazi be in nist chun functionality ye seri ha dar header tarif shode

// Bug : if the robot is in the danger zone the start point can not expand and we get and error
// Bug : what if the goal is in danger zone
// Think: Why do I need to have g_value both in vertex and g_value_parent_pair variable???
// Priority queue of g_value and index  and a vector of parent and index --> see where it goes

// https://stackoverflow.com/questions/21933029/dijkstras-algorithm-how-could-a-priority-queue-or-min-heap-be-used
// https://www.topcoder.com/thrive/articles/Power%20up%20C++%20with%20the%20Standard%20Template%20Library%20Part%20Two:%20Advanced%20Uses

#include<pluginlib/class_list_macros.h>
#include<Dijkstra.h>
PLUGINLIB_EXPORT_CLASS(global_planner::Dijkstra, nav_core::BaseGlobalPlanner) 

namespace global_planner{

    Dijkstra::Dijkstra(){
        std::cout<<"Constructor \n";

    };


    void Dijkstra::initialize(std::string name , costmap_2d::Costmap2DROS* costmap_ros)
    {
        this->costmap_ros = costmap_ros;

        std_msgs::ColorRGBA color; color.a = 1; color.g = 1;
        std_msgs::ColorRGBA color2; color2.a = 1; color2.r = 1;        

        points = std::make_unique<PointWrapper>(color,"map");
        points2 = std::make_unique<PointWrapper>(color2 , "map");

        g_value = std::vector<unsigned int>( 987654321 , 32768);
        visited = std::vector<unsigned int>(32768 , 0);
        parent = std::vector<unsigned int>(32768 , -1);

    }

    bool Dijkstra::makePlan(const geometry_msgs::PoseStamped& start,  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
    {
    /************************************Dijkstra implementation*************************************************************************/
        ROS_ERROR("DIJKSTRA STARTS");
        points->initialize(); // Clear the previous points
        points2->initialize();
        points->publish();
        points2->publish();
        geometry_msgs::Point p;


        unsigned int start_cell_x , start_cell_y , goal_cell_x , goal_cell_y;
        costmap_ros->getCostmap()->worldToMap(start.pose.position.x , start.pose.position.y ,start_cell_x , start_cell_y);
        unsigned int start_index = costmap_ros->getCostmap()->getIndex(start_cell_x , start_cell_y);

        costmap_ros->getCostmap()->worldToMap(goal.pose.position.x , goal.pose.position.y , goal_cell_x , goal_cell_y);
        unsigned int goal_index = costmap_ros->getCostmap()->getIndex(goal_cell_x , goal_cell_y);


        g_value.at(start_index) = 0;
        parent.at(start_index) = -2;

        // Initialize open_list 
        open_list = std::priority_queue<std::pair<unsigned int,unsigned int> , std::vector<std::pair<unsigned int, unsigned int>> , std::greater<std::pair<unsigned int, unsigned int>>>();
        open_list.push(std::pair<unsigned int, unsigned int>( 0 , start_index));

        while (!open_list.empty())
        {
            auto top = open_list.top();
            open_list.pop();
            unsigned int v = top.second , d = top.first;

            if (v == goal_index)
            {
                break;
            } 


            if(d <= g_value[v] && visited[v]==false)
            {
                std::vector<unsigned int>  valid_neighbors_indices;        
                validNeighbors(v, valid_neighbors_indices);
                for (auto& n_index : valid_neighbors_indices)
                {
                    unsigned int v2 = n_index , cost = 1;
                    if(g_value[v2] > g_value[v] + cost)
                    {
                        g_value[v2] = g_value[v] + cost;
                        open_list.push(std::pair<unsigned int , unsigned int>(g_value[v2] , v2));
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



        ROS_ERROR("EDNNNNNNNNNNNNNNND");
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////                
        
        g_value = std::vector<unsigned int>( 987654321 , 32768); // Reinitialize again for the next goal --> I put it here for performance
        visited = std::vector<unsigned int>(32768 , 0);
        parent = std::vector<unsigned int>(32768 , -1);

        return true;
    }




    void Dijkstra::validNeighbors(unsigned int current_node_index , std::vector<unsigned int>& neighbors)
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

