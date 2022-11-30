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

// x and y position of a center of a grid with index
geometry_msgs::Point CostmapUtility::cell_to_pose(const int& index)
{
    geometry_msgs::Point p;
    int cell_y = index / this->costmap_ptr->info.width;
    int cell_x = fmod(index , this->costmap_ptr->info.width);
    p.x = costmap_ptr->info.origin.position.x + costmap_ptr->info.resolution * cell_x + (costmap_ptr->info.resolution /2);
    p.y = costmap_ptr->info.origin.position.y + costmap_ptr->info.resolution * cell_y + (costmap_ptr->info.resolution /2);

    return p;
}

// We have an arbitrary x and y inside of the map w.r.t the odom frame that gazebo creates ---> how can we know which cell this position corresponds to in a 1D array data?
int CostmapUtility::pose_to_cell(const geometry_msgs::Point& position){
    int cell_x = (position.x - costmap_ptr->info.origin.position.x)/ this->costmap_ptr->info.resolution;
    int cell_y = (position.y - costmap_ptr->info.origin.position.y)/ this->costmap_ptr->info.resolution;

    return (cell_y*this->costmap_ptr->info.width + cell_x);
}