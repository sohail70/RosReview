#include<ros/ros.h>
#include<custom_msg/soso.h>

int main(int argc , char** argv)
{
    ros::init(argc,argv,"publish_soheil_data");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<custom_msg::soso>("soheil",1000);
    ros::Rate loop_rate(2);
    custom_msg::soso var;
    var.greeting = "salam";
    var.age = 31;

    while(ros::ok)
    {
        pub.publish(var);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}