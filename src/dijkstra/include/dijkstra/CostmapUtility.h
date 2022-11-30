#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<memory>
#include<vector>

std::vector<std::string> names{"UNKNOWN", "FREESPACE" , "OCCUPIED"};
enum class Grid{
    UNKNOWN = -1,
    FREESPACE = 0,
    OCCUPIED = 100,
};

// Indicies
struct GridType{
    std::unique_ptr<std::vector<int>> unknown_ptr    =  std::make_unique<std::vector<int>>();
    std::unique_ptr<std::vector<int>> free_space_ptr =  std::make_unique<std::vector<int>>();
    std::unique_ptr<std::vector<int>> low_occ_ptr    =  std::make_unique<std::vector<int>>();
    std::unique_ptr<std::vector<int>> high_occ_ptr   =  std::make_unique<std::vector<int>>();
};

class CostmapUtility
{
    public:
        CostmapUtility();
        void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& data);

        void extract();


    private:
        ros::NodeHandle nh;
        ros::Subscriber costmap_sub;
        nav_msgs::OccupancyGrid::ConstPtr costmap_ptr;
        GridType gridType;


};