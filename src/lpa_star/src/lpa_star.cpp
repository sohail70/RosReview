#include<ros/ros.h>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include<vector>
int main(int argc , char** argv)
{
    ros::init(argc, argv , "model");
    std::vector<gazebo::msgs::Model*> models;
    // gazebo::physics::ModelPtr model;
    // model->Load()
}