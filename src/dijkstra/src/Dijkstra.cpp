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
        ROS_INFO("myDIJKSTRA planning a route from start pose to goal pose ... ");
        // ROS_ERROR("EROOOR %s" ,costmap->getName().c_str());
        geometry_msgs::PoseStamped robotPose;
        costmap_ros->getRobotPose(robotPose);  //ino dar cmake yadet nare be library myDijkstra ---> /opt/ros/noetic/lib/libcostmap_2d.so --> alabte vase func e getname niazi be in nist chun functionality ye seri ha dar header tarif shode 
        auto costmap  = costmap_ros->getCostmap();
        ROS_ERROR("posex: %f posey: %f",robotPose.pose.position.x , robotPose.pose.position.y);

        unsigned int cell_x , cell_y;
        costmap->indexToCells(511 , cell_x , cell_y); //0--255   256---511   512---(512+256)     a---(a+map.width)
        ROS_ERROR("cell_x: %i  cell_y: %i" , cell_x , cell_y);

        // ROS_ERROR("POSE X: %f POSE Y: %f",cUtil.cell_to_pose(511).x , cUtil.cell_to_pose(511).y);

        costmap->worldToMap(robotPose.pose.position.x , robotPose.pose.position.y ,cell_x , cell_y );
        ROS_ERROR("Start cell x :  %i Start cell y: %i" , cell_x , cell_y);


        std_msgs::ColorRGBA color; color.a = 1; color.g = 1;
        
        ros::Rate loop_rate(1);

        points = new PointWrapper(color,"map");
        while(ros::ok())
        {
            
            costmap_ros->getRobotPose(robotPose);
            costmap->worldToMap(robotPose.pose.position.x , robotPose.pose.position.y ,cell_x , cell_y );
            geometry_msgs::Point p;
            costmap->mapToWorld(cell_x , cell_y , p.x , p.y);
            points->addPoint(p);
            points->publish();
            ROS_ERROR("PUBLISHING POINTS");
            ros::spinOnce();
            loop_rate.sleep();

        }

        delete points;

        
    /*************************************************************************************************************/
        
        return true;
    }

    void Dijkstra::show(){
        ROS_INFO("Dijkstra Algorithm");
    }



}

