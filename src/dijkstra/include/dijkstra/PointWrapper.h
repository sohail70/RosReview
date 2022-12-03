#include<ros/ros.h>
#include<iostream>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include<std_msgs/ColorRGBA.h>

class PointWrapper{
    public:
        PointWrapper(std_msgs::ColorRGBA  color, std::string frame_id);
        void addPoint(geometry_msgs::Point point);
        void deletePoint(int i );
        void publish();

    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        visualization_msgs::Marker points;
        
        static int id;

};
int PointWrapper::id = 0;
