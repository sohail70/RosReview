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
    {}

    bool Dijkstra::makePlan(const geometry_msgs::PoseStamped& start,  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
    {
        plan.push_back(start);
        for(int i=0 ; i<10 ; i++)
        {
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = 0+(0.5*i);
            // new_goal.pose.position.y = 0+(0.05*i);
            new_goal.pose.position.y = 0.0;

            new_goal.pose.orientation.x = goal_quat.getX();
            new_goal.pose.orientation.y = goal_quat.getY();
            new_goal.pose.orientation.z = goal_quat.getZ();
            new_goal.pose.orientation.w = goal_quat.getW();
            
            plan.push_back(new_goal);
            ROS_INFO("X: %f",new_goal.pose.position.x);
        }
        plan.push_back(goal);
        return true;
    }

}

