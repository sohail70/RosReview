#include<CostmapUtility.h>


CostmapUtility::CostmapUtility(){
    costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid> ("/move_base/global_costmap/costmap" , 1 ,&CostmapUtility::costmap_callback , this );
}



void CostmapUtility::costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
    this->costmap_ptr = data;
}


void CostmapUtility::extract()
{
    for (unsigned int i= 0 ; i < this->costmap_ptr->data.size() ; i++)
    {
        if (this->costmap_ptr->data.at(i) == static_cast<int>(Grid::FREESPACE) )
        {
            this->gridType.free_space_ptr->push_back(i);
        }
        else if (this->costmap_ptr->data.at(i) < static_cast<int>(Grid::OCCUPIED))
        {
            this->gridType.low_occ_ptr->push_back(i);
        }
        else if (this->costmap_ptr->data.at(i) == static_cast<int>(Grid::OCCUPIED))
        {
            this->gridType.high_occ_ptr->push_back(i);
        }
        else if (this->costmap_ptr->data.at(i) == static_cast<int>(Grid::UNKNOWN))
        {
            this->gridType.unknown_ptr->push_back(i);
        }
        else
        {
            ROS_WARN("Unknown cost map value!!!");
        }
    }
}