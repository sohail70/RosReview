#ifndef VISUALIZATION_WRAPPER
#define VISUALIZATION_WRAPPER

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
        void initialize();
        void publish();

    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        visualization_msgs::Marker points;
        
        static int id;

};
int PointWrapper::id = 0;



class LineStripWrapper{
    public:
        LineStripWrapper(std_msgs::ColorRGBA color , std::string frame_id)
        {
            marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 10);
            lines.color = color;    
            lines.header.frame_id = frame_id;
            lines.header.stamp = ros::Time::now();
            lines.pose.orientation.w = 1.0;
            lines.id = id;
            lines.action = visualization_msgs::Marker::ADD;
            lines.scale.x = 0.1;
            lines.type = visualization_msgs::Marker::LINE_STRIP;
            id++;
        }
        void addPointToLineObject(geometry_msgs::Point p)
        {
            lines.points.push_back(p);
        }

        void removePointFromLineObject(int index)
        {
            lines.points.erase(lines.points.begin()+index);   
        }

        void publish()
        {
            marker_pub.publish(lines);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        visualization_msgs::Marker lines;

        static int id;
};
int LineStripWrapper::id = 0;




// creates a line segment with pairs of consecutive points ---> so for each two points there is a line segment and its discontinuous but you can make it continuous like line strip if you make a line for p1 and p2 and then make a line for p2 and p3
class LineListWrapper{
    public:
        LineListWrapper(std_msgs::ColorRGBA color , std::string frame_id)
        {
            marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 10);
            lines.color = color;    
            lines.header.frame_id = frame_id;
            lines.header.stamp = ros::Time::now();
            lines.pose.orientation.w = 1.0;
            lines.id = id;
            lines.action = visualization_msgs::Marker::ADD;
            lines.scale.x = 0.1;
            lines.type = visualization_msgs::Marker::LINE_LIST;
            id++;
        }

        void addLineSegment(geometry_msgs::Point p1 , geometry_msgs::Point p2)
        {
            lines.points.push_back(p1);
            lines.points.push_back(p2);
        }

        void removeLine(int index ) 
        {
            lines.points.erase(lines.points.begin()+index , lines.points.begin()+index+1);
        }

        void publish()
        {
            marker_pub.publish(lines);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        visualization_msgs::Marker lines;

        static int id;
};
int LineListWrapper::id = 0;

#endif