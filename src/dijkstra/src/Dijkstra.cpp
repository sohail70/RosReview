// follow the rules here http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS

#include<pluginlib/class_list_macros.h>
#include<Dijkstra.h>
PLUGINLIB_EXPORT_CLASS(global_planner::Dijkstra, nav_core::BaseGlobalPlanner) 

namespace global_planner{

    Dijkstra::Dijkstra(){
        std::cout<<"Constructor \n";

    };
    Dijkstra::Dijkstra(std::string name_ , costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name,costmap_ros);

    }


    void Dijkstra::initialize(std::string name , costmap_2d::Costmap2DROS* costmap_ros)
    {
        this->costmap_ros = costmap_ros;
    }

    bool Dijkstra::makePlan(const geometry_msgs::PoseStamped& start,  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
    {
        // show();
        // plan.push_back(start);
        // for(int i=0 ; i<10 ; i++)
        // {
        //     geometry_msgs::PoseStamped new_goal = goal;
        //     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        //     new_goal.pose.position.x = 0+(0.5*i);
        //     // new_goal.pose.position.y = 0+(0.05*i);
        //     new_goal.pose.position.y = 0.0;

        //     new_goal.pose.orientation.x = goal_quat.getX();
        //     new_goal.pose.orientation.y = goal_quat.getY();
        //     new_goal.pose.orientation.z = goal_quat.getZ();
        //     new_goal.pose.orientation.w = goal_quat.getW();
            
        //     plan.push_back(new_goal);
        //     ROS_INFO("X: %f",new_goal.pose.position.x);
        // }
        // plan.push_back(goal);
        // return true;
    /*******************************Testing the costmap 2d ros functions*****************************************/
        // ROS_INFO("myDIJKSTRA planning a route from start pose to goal pose ... ");
        // // ROS_ERROR("EROOOR %s" ,costmap->getName().c_str());
        // geometry_msgs::PoseStamped robotPose;
        // costmap_ros->getRobotPose(robotPose);  //ino dar cmake yadet nare be library myDijkstra ---> /opt/ros/noetic/lib/libcostmap_2d.so --> alabte vase func e getname niazi be in nist chun functionality ye seri ha dar header tarif shode 
        // auto costmap  = costmap_ros->getCostmap();
        // ROS_ERROR("posex: %f posey: %f",robotPose.pose.position.x , robotPose.pose.position.y);

        // unsigned int cell_x , cell_y;
        // costmap->indexToCells(511 , cell_x , cell_y); //0--255   256---511   512---(512+256)     a---(a+map.width)
        // ROS_ERROR("cell_x: %i  cell_y: %i" , cell_x , cell_y);

        // // ROS_ERROR("POSE X: %f POSE Y: %f",cUtil.cell_to_pose(511).x , cUtil.cell_to_pose(511).y);

        // costmap->worldToMap(robotPose.pose.position.x , robotPose.pose.position.y ,cell_x , cell_y );
        // ROS_ERROR("Start cell x :  %i Start cell y: %i" , cell_x , cell_y);


        std_msgs::ColorRGBA color; color.a = 1; color.g = 1;
        
        // ros::Rate loop_rate(1);

        points = new PointWrapper(color,"map");
        // while(ros::ok())
        // {
            
        //     costmap_ros->getRobotPose(robotPose);
        //     costmap->worldToMap(robotPose.pose.position.x , robotPose.pose.position.y ,cell_x , cell_y );
            geometry_msgs::Point p;
        //     costmap->mapToWorld(cell_x , cell_y , p.x , p.y);
        //     points->addPoint(p);
        //     points->publish();
        //     ROS_ERROR("PUBLISHING POINTS");
        //     ros::spinOnce();
        //     loop_rate.sleep();

        // }

        // delete points;

        
    /************************************Dijkstra implementation*************************************************************************/
        
        Vertex goal_node; goal_node.status=static_cast<int>(global_planner::status::UNCHECKED);
        Vertex start_node; start_node.g_value = 0; start_node.parent = nullptr;
        costmap_ros->getCostmap()->worldToMap(goal.pose.position.x , goal.pose.position.y ,goal_node.cell_x ,goal_node.cell_y ); 
        costmap_ros->getCostmap()->worldToMap(start.pose.position.x , start.pose.position.y ,start_node.cell_x , start_node.cell_y);


        open_list.push(start_node);
 
        Vertex current_node;
        std::vector<Vertex> current_neighbors;        
        // auto size_x = costmap_ros->getCostmap()->getSizeInCellsX(); // width of the map (as in # of cells in y direction)
        // auto size_y = costmap_ros->getCostmap()->getSizeInCellsY(); // height of the map (as in # of cells in x direction)
        // ROS_ERROR("ASDASDASD %i ASDASD %i" , size_x, size_y);
        // ROS_ERROR("STARTING DIJKSTRA");
        ros::Rate loop_rate(20);
        
        while(ros::ok() || current_node != goal_node)
        {
            // ROS_ERROR("IN THE WHILE LOOP");
            current_node  = open_list.top();
            current_node.status = static_cast<int>(global_planner::status::OPEN);

            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , p.x , p.y);
            points->addPoint(p);
            points->publish();

            ROS_ERROR("Cell_x: %i Cell_y: %i" ,current_node.cell_x , current_node.cell_y );
            neighborsIntoList(current_node);
            open_list.pop();
            closed_list.insert(current_node);
            current_node.status = static_cast<int>(global_planner::status::CLOSE);
            loop_rate.sleep(); 
            // points->deletePoint(0);
        }        
        delete points; 

        geometry_msgs::PoseStamped pose;     
        pose.pose.orientation.w = 1;
        current_node = goal_node; 
        while(current_node != start_node)
        {
            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , pose.pose.position.x , pose.pose.position.y);
            plan.push_back(pose);         
        }
        ////////////////////////////////////////////////////////
        // ROS_ERROR("DIJKSTRA COMING THROUGH");
        // unsigned int width =  costmap_ros->getCostmap()->getSizeInCellsX();
        // unsigned int height = costmap_ros->getCostmap()->getSizeInCellsY();

        
        // Vertex v;
        // for (unsigned int i = 0 ; i< width ; i++)
        // {
        //     for(unsigned int j = 0 ; j <height ; j++)
        //     {
        //         if(i == start_node.cell_x && j == start_node.cell_y)
        //             continue;
            
        //         v.cell_x = i ; v.cell_y = j ; v.g_value = std::numeric_limits<float>::infinity(); v.parent = nullptr;
        //         if (costmap_ros->getCostmap()->getCost(i,j)==costmap_2d::FREE_SPACE)
        //             v.cost = 0;
        //         else if (costmap_ros->getCostmap()->getCost(i,j)==costmap_2d::NO_INFORMATION)
        //             v.cost = -1;
        //         else
        //             v.cost = 1;

        //         open_list.push(v);
        //     }
        // }

        // while(!open_list.empty())
        // {
        //     current_node = open_list.top();
        //     open_list.pop();
        //     // validNeighbors()
        // }
         
        return true;
    }

    void Dijkstra::show(){
        ROS_INFO("Dijkstra Algorithm");
    }

    void Dijkstra::neighborsIntoList(Vertex current_node) 
    {
        
        // right
        if( boundaryCheck(current_node.cell_x+1 , current_node.cell_y ) && costmap_ros->getCostmap()->getCost(current_node.cell_x + 1 , current_node.cell_y) ==costmap_2d::FREE_SPACE) // age cell_x va cell_y < 0 beshe irad dare pas badan ye check bezar bara in
        {
            int right_cell_x = current_node.cell_x+1;
            int right_cell_y = current_node.cell_y;

            auto iter = std::find_if(closed_list.begin() , closed_list.end() , [right_cell_x , right_cell_y](Vertex v){return (v.cell_x == right_cell_x && v.cell_y==right_cell_y); });
            if (iter == closed_list.end())
            {
                Vertex v;v.cell_x = current_node.cell_x+1; v.cell_y = current_node.cell_y ; v.parent= &current_node; v.g_value = current_node.g_value+1;
                if(statusCheck(v.status))
                {
                    open_list.push(v);
                } 
            }
        }
        // up
        if( boundaryCheck(current_node.cell_x , current_node.cell_y+1) && costmap_ros->getCostmap()->getCost(current_node.cell_x , current_node.cell_y + 1) == costmap_2d::FREE_SPACE)
        {
            int up_cell_x = current_node.cell_x;
            int up_cell_y = current_node.cell_y+1;

            auto iter = std::find_if(closed_list.begin() , closed_list.end() , [up_cell_x , up_cell_y](Vertex v){return (v.cell_x == up_cell_x && v.cell_y==up_cell_y); });
            if (iter == closed_list.end())
            {
                Vertex v;v.cell_x = current_node.cell_x; v.cell_y = current_node.cell_y+1 ; v.parent = &current_node; v.g_value = current_node.g_value+1; 
                if(statusCheck(v.status))
                {
                    open_list.push(v);
                }
            }
        }
        // left
        if( boundaryCheck(current_node.cell_x - 1 , current_node.cell_y) && costmap_ros->getCostmap()->getCost(current_node.cell_x - 1 , current_node.cell_y) == costmap_2d::FREE_SPACE)
        {
            int left_cell_x = current_node.cell_x-1;
            int left_cell_y = current_node.cell_y;

            auto iter = std::find_if(closed_list.begin() , closed_list.end() , [left_cell_x , left_cell_y](Vertex v){return (v.cell_x == left_cell_x && v.cell_y==left_cell_y); });
            if (iter == closed_list.end())
            {
                Vertex v;v.cell_x = current_node.cell_x-1; v.cell_y = current_node.cell_y ; v.parent = &current_node; v.g_value = current_node.g_value+1;
                if(statusCheck(v.status))
                {
                    open_list.push(v);
                }
            }
        }
        // down
        if( boundaryCheck(current_node.cell_x , current_node.cell_y - 1) && costmap_ros->getCostmap()->getCost(current_node.cell_x  , current_node.cell_y - 1)== costmap_2d::FREE_SPACE)
        {
            int down_cell_x = current_node.cell_x;
            int down_cell_y = current_node.cell_y-1;

            auto iter = std::find_if(closed_list.begin() , closed_list.end() , [down_cell_x , down_cell_y](Vertex v){return (v.cell_x == down_cell_x && v.cell_y==down_cell_y); });
            if (iter == closed_list.end())
            {
                Vertex v;v.cell_x = current_node.cell_x; v.cell_y = current_node.cell_y-1 ; v.parent = &current_node; v.g_value = current_node.g_value+1; 
                if(statusCheck(v.status))
                {
                    open_list.push(v);
                }
            }
        }
    }

    bool Dijkstra::boundaryCheck(int cell_x  ,int cell_y )
    {
        if (cell_x > costmap_ros->getCostmap()->getSizeInCellsX() ||
            cell_x < 0 ||
            cell_y > costmap_ros->getCostmap()->getSizeInCellsY() ||
            cell_y < 0)
            {return false;} 
        else
            {return true;}
            
    }

    bool Dijkstra::statusCheck(int status)
    {
        if(status == static_cast<int>(global_planner::status::CLOSE))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}

