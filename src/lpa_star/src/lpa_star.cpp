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
        unsigned x , y;

        geometry_msgs::Point p;

        std_msgs::ColorRGBA color1; color1.a =1 ; color1.g = 1;
        std_msgs::ColorRGBA color2; color2.a = 1 ; color2.r = 1;
        points1 = std::make_unique<PointWrapper>(color1, "map");
        points2 = std::make_unique<PointWrapper>(color2, "map");
        for (int i= 0 ; i<98304 ; i++)
        {
            this->costmap_ros->getCostmap()->indexToCells(i , x, y);    
            if(this->costmap_ros->getCostmap()->getCost(x,y)>0)
            {
                ROS_ERROR("cost: %i",this->costmap_ros->getCostmap()->getCost(x,y));
                this->costmap_ros->getCostmap()->mapToWorld(x,y,p.x,p.y); 
                this->points1->addPoint(p);
                

            }
            else
            {
                this->costmap_ros->getCostmap()->mapToWorld(x,y,p.x,p.y); 
                this->points2->addPoint(p);
            }

        } 

    }

    void LpaStar::test()
    {
        this->points1->publish();
        this->points2->publish();
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
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    std::string robot_base_frame_ , global_frame_;
    nh.param("lpa_star/robot_base_frame", robot_base_frame_, std::string("nothing"));
    nh.param("lpa_star/global_frame", global_frame_, std::string("nothing"));
    ROS_ERROR("SSSS %s %s", robot_base_frame_.c_str() , global_frame_.c_str());

    geometry_msgs::PoseStamped pose;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);;
    costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("global_costmap", tfBuffer);
    auto charmap = costmap_ros->getCostmap()->getCharMap();
    for(int i = 0 ; i<98304 ; i++ , charmap++)
    {
        if(*charmap>0)
            ROS_ERROR("ZZZZZ %i" , *charmap);
    }


    costmap_ros->start();
    ROS_ERROR("ASDASD %f %f %f " , costmap_ros->getRobotFootprint().at(0).x , costmap_ros->getRobotFootprint().at(0).y, costmap_ros->getRobotFootprint().at(0).z);
    // costmap_ros->resetLayers();
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

    ros::Rate loop_rate(1);
    while(ros::ok())
    {    

        lpa_star.test();
        loop_rate.sleep();
        ros::spinOnce();
    } 

}