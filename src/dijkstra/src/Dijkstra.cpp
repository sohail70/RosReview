// follow the rules here http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
// TODO: Read https://stackoverflow.com/questions/649640/how-to-do-an-efficient-priority-update-in-stl-priority-queue
// OR use this condition ---> g_value --> age g_value avaliesh inf nabood yani to opne list hast va niazi nist bezari to vali bayad berror beshe va nemishe be priority queue access kard unfortuanetly pas hamoon ravesh bala ro boro
// Ya mishe ye list az visited node ha ham ijad kard ke onam mesle closed_list check beshe! --> be nazaram kheili toolani mishe !!! raveshe efficient fek kunam hamoon balae bashe vali vase shoro ino bezan
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
        std::vector<int>  valid_neighbors_indices;        
        // // auto size_x = costmap_ros->getCostmap()->getSizeInCellsX(); // width of the map (as in # of cells in y direction)
        // // auto size_y = costmap_ros->getCostmap()->getSizeInCellsY(); // height of the map (as in # of cells in x direction)
        // // ROS_ERROR("ASDASDASD %i ASDASD %i" , size_x, size_y);
        // // ROS_ERROR("STARTING DIJKSTRA");
        // ros::Rate loop_rate(20);
        
        // while(ros::ok() || current_node != goal_node)
        // {
        //     // ROS_ERROR("IN THE WHILE LOOP");
        //     current_node  = open_list.top();
        //     current_node.status = static_cast<int>(global_planner::status::OPEN);

        //     costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , p.x , p.y);
        //     points->addPoint(p);
        //     points->publish();

        //     ROS_ERROR("Cell_x: %i Cell_y: %i" ,current_node.cell_x , current_node.cell_y );
        //     neighborsIntoList(current_node);
        //     open_list.pop();
        //     closed_list.insert(current_node);
        //     current_node.status = static_cast<int>(global_planner::status::CLOSE);
        //     loop_rate.sleep(); 
        //     // points->deletePoint(0);
        // }        
        // delete points; 

        // geometry_msgs::PoseStamped pose;     
        // pose.pose.orientation.w = 1;
        // current_node = goal_node; 
        // while(current_node != start_node)
        // {
        //     costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , pose.pose.position.x , pose.pose.position.y);
        //     plan.push_back(pose);         
        // }
        ////////////////////////////////////////////////////////
        ROS_ERROR("DIJKSTRA COMING THROUGH");
        unsigned int width =  costmap_ros->getCostmap()->getSizeInCellsX();
        unsigned int height = costmap_ros->getCostmap()->getSizeInCellsY();

        
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
        ROS_ERROR("BEFORE WHILE");
        double dist_to_current_node = 1;
        ros::Rate loop_rate(100);
        bool reached = false;

        std::vector<Vertex> cur;
        cur.resize(32768);
        while(ros::ok() && !open_list.empty() && !reached)
        {
            current_node = open_list.top();
            if (current_node.cell_x == goal_node.cell_x && current_node.cell_y == goal_node.cell_y)
                break;
            // ROS_ERROR("INTO THE WHILE: i: %i , j:%i" , current_node.cell_x , current_node.cell_y);

            open_list.pop();
            validNeighbors(current_node , valid_neighbors_indices);
            for(auto& ind : valid_neighbors_indices)
            {
                if( current_node.g_value + dist_to_current_node  < g_value_parent_pair.at(ind).first)
                {
                    
                    cur[ind] = (current_node);
                    g_value_parent_pair.at(ind).first = current_node.g_value + dist_to_current_node;
                    g_value_parent_pair.at(ind).second = &cur[ind];
                    unsigned int n_cell_x , n_cell_y;
                    costmap_ros->getCostmap()->indexToCells(ind, n_cell_x , n_cell_y);
                    ROS_ERROR("parent for: %i,%i is cell %i,%i" ,n_cell_x ,n_cell_y , g_value_parent_pair.at(ind).second->cell_x ,g_value_parent_pair.at(ind).second->cell_y ) ;
                    
                    costmap_ros->getCostmap()->indexToCells(ind , v.cell_x , v.cell_y);
                    v.g_value = current_node.g_value + dist_to_current_node; v.parent = &cur[ind];
                    open_list.push(v);

                    // if (n_cell_x == goal_node.cell_x && n_cell_y==goal_node.cell_y)
                    // {
                    //     current_node = goal_node; 
                    //     reached = true;
                    //     break;
                    // }
                }
            }

            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , p.x , p.y);
            points->addPoint(p);
            points->publish();

            ros::spinOnce();
            loop_rate.sleep();
        }
        //57-34  
        std::vector<Vertex> path;
        ros::Rate loop_rate_2(10);
        int index;
        geometry_msgs::PoseStamped pos;
        pos.pose.orientation.w = 1;
        pos.header.frame_id="map";
        while(true)
        // while(current_node.parent==nullptr)
        {
            ROS_ERROR("PATH: %i-%i ",current_node.cell_x , current_node.cell_y);
            index = costmap_ros->getCostmap()->getIndex(current_node.cell_x , current_node.cell_y); 
            ROS_ERROR("INDEX: %i", index);
            costmap_ros->getCostmap()->mapToWorld(current_node.cell_x , current_node.cell_y , pos.pose.position.x , pos.pose.position.y); 
            plan.push_back(pos);
            path.push_back(current_node);
            ROS_ERROR("PUSHED");
            current_node = *g_value_parent_pair.at(index).second;
            ROS_ERROR("CURRENT_SET");
            if ((current_node.cell_x==start_node.cell_x) && (current_node.cell_y == start_node.cell_y))
            {
                ROS_ERROR("REACHED STARTING POINT");
                break;
            }
            ros::spinOnce();
            loop_rate_2.sleep();
        }
        path.push_back(start_node);
        std::reverse(path.begin() , path.end());
        std::reverse(plan.begin() , plan.end()); 
        ROS_ERROR("ENNNNNNNNNNNNNNNND");
        return true;
    }

    void Dijkstra::show(){
        ROS_INFO("Dijkstra Algorithm");
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
                // if(statusCheck(v.status))
                // {
                    open_list.push(v);
                // } 
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
                // if(statusCheck(v.status))
                // {
                    open_list.push(v);
                // }
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
                // if(statusCheck(v.status))
                // {
                    open_list.push(v);
                // }
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
                // if(statusCheck(v.status))
                // {
                    open_list.push(v);
                // }
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

