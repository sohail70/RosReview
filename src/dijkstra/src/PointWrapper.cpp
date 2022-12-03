#include<PointWrapper.h>

PointWrapper::PointWrapper(std_msgs::ColorRGBA color , std::string frame_id ){
    marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 10);
    points.color = color;    
    points.header.frame_id = frame_id;
    points.header.stamp = ros::Time::now();
    points.pose.orientation.w = 1.0;
    points.id = id;
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.type = visualization_msgs::Marker::POINTS;

    id++;
}


void PointWrapper::addPoint(geometry_msgs::Point point)
{
    points.points.push_back(point);
}
void PointWrapper::deletePoint(int i )
{
    points.points.erase(points.points.begin()+i);
}
void PointWrapper::publish()
{
    marker_pub.publish(points);
}
