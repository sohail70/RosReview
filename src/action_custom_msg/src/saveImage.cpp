#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include<opencv4/opencv2/highgui.hpp>
void save(const sensor_msgs::Image::ConstPtr& data)
{
    cv::Mat image;
    image = cv_bridge::toCvShare(data,"rgb8")->image;
    

    static int i = 0;
    std::string soheil = "soheil" + std::to_string(i)+".jpg";
    i++;
    ROS_INFO("saving image: %s",soheil.c_str());

    // imgage write
    bool b = cv::imwrite("/home/sohail/img/"+soheil,image);
    // cv::waitKey(1000);
    ros::Duration(1).sleep(); //farghi ba balee nadare--> on male open cv hast in male ros ke hardo ye had daran


    // // image show
    // cv::namedWindow("image");
    // cv::imshow("image",image);
    // cv::waitKey(30);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv , "saveImage");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/front_cam/camera/image",1,save); //queue 1 moheme! vagarna akshaye ziadtari write mishe!
    
    ros::spin();
}