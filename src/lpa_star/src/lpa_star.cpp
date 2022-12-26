//384 X 256 0.1 resolution
#include<ros/ros.h>
#include<lpa_star/lpa_star.h>


namespace DynamicPlanner{
    LpaStar::LpaStar(){
       
    }

    void LpaStar::initialize(const costmap_2d::Costmap2D& costmap)
    {

    }


    bool LpaStar::makePlan(const geometry_msgs::PoseStamped& start , const geometry_msgs::PoseStamped& goal , std::vector<geometry_msgs::PoseStamped> plan)
    {

    }


}






#include<geometry_msgs/PoseStamped.h>
//https://answers.ros.org/question/332304/problem-with-costmap2dros-and-tf-transform-listener-in-ros-melodic/
costmap_2d::Costmap2DROS* lcr ;

int main(int argc , char** argv)
{
    ros::init(argc, argv , "global_costmap");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped pose;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);;
    lcr = new costmap_2d::Costmap2DROS("costmap", tfBuffer);
    lcr->start();
    // ROS_ERROR("ASDASD %f %f %f " , lcr->getRobotFootprint().at(0).x , lcr->getRobotFootprint().at(0).y, lcr->getRobotFootprint().at(0).z);
    
    while(ros::ok())
    {

        lcr->getRobotPose(pose);
        ROS_ERROR("POSE: %f %f" , pose.pose.position.x , pose.pose.position.y);
        ros::spinOnce();
    }
}