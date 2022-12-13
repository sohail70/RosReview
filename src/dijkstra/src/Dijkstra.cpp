// follow the rules here http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
// Note : Read https://stackoverflow.com/questions/649640/how-to-do-an-efficient-priority-update-in-stl-priority-queue
// baraye use kardan az func haye costmap2d_ ros  ----> ino dar cmake yadet nare be library myDijkstra ---> /opt/ros/noetic/lib/libcostmap_2d.so --> alabte vase func e getname niazi be in nist chun functionality ye seri ha dar header tarif shode

// Bug : if the robot is in the danger zone the start point can not expand and we get and error
// Bug : what if the goal is in danger zone
// Think: Why do I need to have g_value both in vertex and g_value_parent_pair variable???
// Priority queue of g_value and index  and a vector of parent and index --> see where it goes
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

    }

    bool Dijkstra::makePlan(const geometry_msgs::PoseStamped& start,  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
    {
    /************************************Dijkstra implementation*************************************************************************/
        points->initialize(); // Clear the previous points
        points2->initialize();
        points->publish();
        points2->publish();
        geometry_msgs::Point p;

                
        
        Vertex goal_node; 
        Vertex start_node; start_node.g_value = 0; 
        costmap_ros->getCostmap()->worldToMap(goal.pose.position.x , goal.pose.position.y ,goal_node.cell_x ,goal_node.cell_y ); 
        costmap_ros->getCostmap()->worldToMap(start.pose.position.x , start.pose.position.y ,start_node.cell_x , start_node.cell_y);

        open_list = std::priority_queue<Vertex , std::vector<Vertex> , Order>(); // to clear the last open_list for new goal
        g_value_parent_pair.clear();
        open_list.push(start_node);
        std::vector<int>  valid_neighbors_indices;        

        ////////////////////////////////////////////////////////
        ROS_ERROR("DIJKSTRA COMING THROUGH");
        unsigned int width =  costmap_ros->getCostmap()->getSizeInCellsX(); // width of the map (as in # of cells in y direction)
        unsigned int height = costmap_ros->getCostmap()->getSizeInCellsY(); // height of the map (as in # of cells in x direction)

        Vertex v;
        for (unsigned int i = 0 ; i< height ; i++)
        {
            for(unsigned int j = 0 ; j <width ; j++)
            {
                if(i == start_node.cell_y && j == start_node.cell_x)
                {
                    g_value_parent_pair.push_back(std::make_pair(0 ,nullptr ));
                    continue;
                }
                g_value_parent_pair.push_back(std::make_pair(std::numeric_limits<float>::infinity() ,nullptr ));
            }
        }

        double dist_to_current_node = 1; // We are not dealing with weighted graph so this variable has value 1
        std::vector<Vertex> nodes;
        nodes.resize(width*height);
        Vertex current_node;
        while(!open_list.empty()) 
        {
            current_node = open_list.top();
            if (current_node.cell_x == goal_node.cell_x && current_node.cell_y == goal_node.cell_y)
                break;
            int current_node_ind = costmap_ros->getCostmap()->getIndex(current_node.cell_x , current_node.cell_y);
            open_list.pop();
            validNeighbors(current_node , valid_neighbors_indices);
            for(auto& ind : valid_neighbors_indices)
            {
                if(g_value_parent_pair.at(current_node_ind).first + dist_to_current_node  < g_value_parent_pair.at(ind).first)
                {
                    nodes[ind] = current_node;
                    g_value_parent_pair.at(ind).first = g_value_parent_pair.at(current_node_ind).first + dist_to_current_node;
                    g_value_parent_pair.at(ind).second = &nodes[ind];
                    unsigned int n_cell_x , n_cell_y;
                    costmap_ros->getCostmap()->indexToCells(ind, n_cell_x , n_cell_y);
                    
                    costmap_ros->getCostmap()->indexToCells(ind , v.cell_x , v.cell_y);
                    v.g_value = current_node.g_value + dist_to_current_node; 
                    open_list.push(v);

                }
            }

            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , p.x , p.y);
            points->addPoint(p);
            points->publish();
        }

        geometry_msgs::PoseStamped pos;
        pos.pose.orientation.w = 1;
        pos.header.frame_id="map";

        while(true)
        {
            int index = costmap_ros->getCostmap()->getIndex(current_node.cell_x , current_node.cell_y); 
            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , pos.pose.position.x , pos.pose.position.y); 
            plan.push_back(pos);
            
            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , p.x , p.y);
            points2->addPoint(p);
            points2->publish();


            current_node = *g_value_parent_pair.at(index).second;
            if ((current_node.cell_x==start_node.cell_x) && (current_node.cell_y == start_node.cell_y))
            {
                break;
            }
        }

        plan[0].pose.orientation = goal.pose.orientation; 
        std::reverse(plan.begin() , plan.end()); 
        ROS_ERROR("Finish Planning a Route");
        return true;
    }




    void Dijkstra::validNeighbors(Vertex current_node , std::vector<int>& neighbors)
    {
        // right
        int cell_x_right = current_node.cell_x + 1;
        int cell_y_right = current_node.cell_y; 
        if(costmap_ros->getCostmap()->getCost(cell_x_right , cell_y_right) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_right , cell_y_right);
            neighbors.push_back(index);   
        } 

        // up
        int cell_x_up = current_node.cell_x; 
        int cell_y_up = current_node.cell_y + 1;
        if(costmap_ros->getCostmap()->getCost(cell_x_up , cell_y_up) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_up , cell_y_up);
            neighbors.push_back(index);   
        }  

        // left
        int cell_x_left = current_node.cell_x - 1;
        int cell_y_left = current_node.cell_y;
        if(costmap_ros->getCostmap()->getCost(cell_x_left , cell_y_left) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_left , cell_y_left);
            neighbors.push_back(index);   
        } 

        // down
        int cell_x_down = current_node.cell_x;
        int cell_y_down = current_node.cell_y - 1;
        if(costmap_ros->getCostmap()->getCost(cell_x_down , cell_y_down) == costmap_2d::FREE_SPACE)
        {
            int index = costmap_ros->getCostmap()->getIndex(cell_x_down , cell_y_down);
            neighbors.push_back(index);   
        } 

    }

}

