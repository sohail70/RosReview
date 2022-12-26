//384 X 256 0.1 resolution
#include<ros/ros.h>
#include<lpa_star/lpa_star.h>


namespace DynamicPlanner{
    LpaStar::LpaStar(){
       
    }

    LpaStar::~LpaStar(){}

    void LpaStar::initialize(const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
    {
        this->costmap_ros = costmap_ros;
    }


    bool LpaStar::makePlan(const geometry_msgs::PoseStamped& start , const geometry_msgs::PoseStamped& goal , std::vector<geometry_msgs::PoseStamped> plan)
    {

    }


}



void func(const costmap_2d::Costmap2DROS& object)
{

}




//https://answers.ros.org/question/332304/problem-with-costmap2dros-and-tf-transform-listener-in-ros-melodic/
std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros ;

int main(int argc , char** argv)
{
    
    ros::init(argc, argv , "lpa_star");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped pose;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);;
    costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("costmap", tfBuffer);
    costmap_ros->start();
    ROS_ERROR("ASDASD %f %f %f " , costmap_ros->getRobotFootprint().at(0).x , costmap_ros->getRobotFootprint().at(0).y, costmap_ros->getRobotFootprint().at(0).z);

    // ros::Rate loop_rate(1); 
    // while(ros::ok())
    // {
        
    //     costmap_ros->getRobotPose(pose);
    //     ROS_ERROR("POSE: %f %f" , pose.pose.position.x , pose.pose.position.y);
    //     ros::spinOnce();

    //     loop_rate.sleep();
    // }


    DynamicPlanner::LpaStar lpa_star;
    lpa_star.initialize(costmap_ros);
    ros::spin();
    

}