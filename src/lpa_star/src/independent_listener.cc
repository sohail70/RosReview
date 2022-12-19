#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include<ros/ros.h>
#include<lpa_star/dynamic_obs.h>

/////////////////////////////////////////////////
ros::Publisher pub;
lpa_star::dynamic_obs obs_info;


// Function is called every time a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  // std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
  // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    if (name == std::string("box_1") || name == std::string("box_2") || name == std::string("box_3") || name == std::string("box_4") )
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();

      double x = position.x();
      double y = position.y();
      double z = position.z();

      // std::cout << "Read position: x: " << x
      //     << " y: " << y << " z: " << z << std::endl;
      
      obs_info.frame_id = "odom";
      obs_info.name = name;
      obs_info.x_center = x;
      obs_info.y_center = y;
      // ROS_ERROR( "READING %s %f %f", obs_info.name.c_str() , obs_info.x_center , obs_info.y_center);
      pub.publish(obs_info);
    }
  }

  
}


/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo pose info topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);


  gazebo::transport::NodePtr node2(new gazebo::transport::Node());
  node2->Init();



  ros::init(_argc ,_argv , "obstacle_pose");
  ros::NodeHandle nh;
  pub = nh.advertise<lpa_star::dynamic_obs>("obstacle_position", 1000);


  // Busy wait loop...replace with your own code as needed.
  while (ros::ok()) //ros::ok ro age jash true bezari vaghti ctrl+c mizani vasate run kar nemikune !!
  {
    gazebo::common::Time::MSleep(1000);
    ROS_ERROR("IN The While");
    ros::spinOnce();
  }
  

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
