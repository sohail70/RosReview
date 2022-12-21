#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <lpa_star/dynamic_obs.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/Point32.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      int argc; char** argv;
      ros::init(argc,argv , "box_size");
      pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("obstacles",10);

      this->model = _parent;

      this->lastUpdate = gazebo::common::Time(0);


      // gazebo::physics::BasePtr link = model->GetChild(0); //we get the link but its still the basePtr - so below we cast it 
      gazebo::physics::LinkPtr link =  model->GetLink("link");
      // gazebo::physics::CollisionPtr collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(link->GetChild(0));
      gazebo::physics::CollisionPtr collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(link->GetCollision("collision"));
      gazebo::physics::ShapePtr shape(collision->GetShape());
      gazebo::physics::BoxShapePtr box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape);
      ignition::math::Vector3d  vec3 = box->Size();
      box_size.push_back(vec3[0]);box_size.push_back(vec3[1]); box_size.push_back(vec3[2]);

      try{
        nh.setParam(this->model->GetName()+"_size" ,box_size );
      }
      catch(const std::exception& e)
      {
        std::cout<<"Exception: "<<e.what()<<"\n";
      }

      double x , y; 
      if (_parent->GetName()=="box_1")
      {
        x=5;y=5;
        lastPose1.SetX(x);
        lastPose1.SetY(y);
      }
      else if (_parent->GetName()=="box_2")
      {
        x=10;y=-5;
        lastPose2.SetX(x);
        lastPose2.SetY(y);
      }
      else if(_parent->GetName()=="box_3")
      {
        x=15;y=5;
        lastPose3.SetX(x);
        lastPose3.SetY(y);
      }
      else if(_parent->GetName()=="box_4")
      {
        x=20;y=-5;
        lastPose4.SetX(x);
        lastPose4.SetY(y);
      }

      gazebo::common::PoseAnimationPtr anim(
            new gazebo::common::PoseAnimation("test", 40.0, true));

      gazebo::common::PoseKeyFrame *key;

      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->Translation(ignition::math::Vector3d(x, -y, 0));
      key->Rotation(ignition::math::Quaterniond(0, 0, 0));

      // set waypoint location after 2 seconds
      key = anim->CreateKeyFrame(20.0);
      key->Translation(ignition::math::Vector3d(x, +y, 0));
      key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

      // set final location equal to starting location
      key = anim->CreateKeyFrame(40.0);
      key->Translation(ignition::math::Vector3d(x, -y, 0));
      key->Rotation(ignition::math::Quaterniond(0, 0, 0));


      obstacles_info.obstacles.resize(4);
      obstacles_info.obstacles.at(0).polygon.points.resize(1);
      obstacles_info.obstacles.at(1).polygon.points.resize(1);
      obstacles_info.obstacles.at(2).polygon.points.resize(1);
      obstacles_info.obstacles.at(3).polygon.points.resize(1);

      // set the animation
      _parent->SetAnimation(anim);

      // https://answers.gazebosim.org//question/19883/can-someone-explain-the-role-of-connectworldupdatebegin-function/
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AnimatedBox::OnUpdate, this,std::placeholders::_1));
    
    }
    public: void OnUpdate(const common::UpdateInfo& info) //https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot
    {
      // ROS_WARN("This function is called every time! I don't need it now"); //Maybe I can use it to publish velocity and positions to obstacle topic for the teb local planner
      double dt = (info.simTime - this->lastUpdate).Double();
      // ROS_ERROR("DDDDTTTTT: %f",dt);
      
      gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->model->GetLink("link"));
      // ROS_ERROR("Name: %s %f %f %f" ,this->model->GetName().c_str() ,  pose.X() , pose.Y() , pose.Z());
      

      obstacles_info.header.frame_id = "odom";
      obstacles_info.header.stamp = ros::Time::now();

      

      std::string name = this->model->GetName();
      static int counter = 0;
      if (name=="box_1")
      {
        this->Pose1 = link->WorldCoGPose();

        obstacles_info.obstacles.at(0).velocities.twist.linear.y = (Pose1.Y() - this->lastPose1.Y()) / dt;
        obstacles_info.obstacles.at(0).id = 1; //box_1

        obstacles_info.obstacles.at(0).polygon.points[0].x = Pose1.X();
        obstacles_info.obstacles.at(0).polygon.points[0].y = Pose1.Y();
        obstacles_info.obstacles.at(0).polygon.points[0].z = Pose1.Z();

        // ROS_ERROR("Name: %s %f %f %f", name.c_str(), Pose1.Y() , this->lastPose1.Y() ,  obstacles_info.obstacles.at(0).velocities.twist.linear.y);
        this->lastPose1.SetY(Pose1.Y());
        counter++;
 
      }
      else if (name=="box_2")
      {
        this->Pose2 = link->WorldCoGPose();

        obstacles_info.obstacles.at(1).velocities.twist.linear.y = (Pose2.Y() - this->lastPose2.Y()) / dt;
        obstacles_info.obstacles.at(1).id = 2;

        obstacles_info.obstacles.at(1).polygon.points[0].x = Pose2.X();
        obstacles_info.obstacles.at(1).polygon.points[0].y = Pose2.Y();
        obstacles_info.obstacles.at(1).polygon.points[0].z = Pose2.Z();
        // ROS_ERROR("Name: %s %f %f %f", name.c_str(), Pose2.Y() , this->lastPose2.Y() ,  obstacles_info.obstacles.at(1).velocities.twist.linear.y);
        this->lastPose2.SetY(Pose2.Y());
        counter++;
      }
      else if (name=="box_3")
      {
        this->Pose3 = link->WorldCoGPose();

        obstacles_info.obstacles.at(2).velocities.twist.linear.y = (Pose3.Y() - this->lastPose3.Y()) / dt;
        obstacles_info.obstacles.at(2).id = 3;

        obstacles_info.obstacles.at(2).polygon.points[0].x = Pose3.X();
        obstacles_info.obstacles.at(2).polygon.points[0].y = Pose3.Y();
        obstacles_info.obstacles.at(2).polygon.points[0].z = Pose3.Z();
        // ROS_ERROR("Name: %s %f %f %f", name.c_str(), Pose3.Y() , this->lastPose3.Y() ,  obstacles_info.obstacles.at(2).velocities.twist.linear.y);
        this->lastPose3.SetY(Pose3.Y());
        counter++;
      }
      else if (name=="box_4")
      {
        this->Pose4 = link->WorldCoGPose();

        obstacles_info.obstacles.at(3).velocities.twist.linear.y = (Pose4.Y() - this->lastPose4.Y()) / dt;
        obstacles_info.obstacles.at(3).id = 4;

        obstacles_info.obstacles.at(3).polygon.points[0].x = Pose4.X();
        obstacles_info.obstacles.at(3).polygon.points[0].y = Pose4.Y();
        obstacles_info.obstacles.at(3).polygon.points[0].z = Pose4.Z();
        // ROS_ERROR("Name: %s %f %f %f", name.c_str(), Pose4.Y() , this->lastPose4.Y() ,  obstacles_info.obstacles.at(3).velocities.twist.linear.y);
        this->lastPose4.SetY(Pose4.Y());
        counter++;
      }




      this->lastUpdate = info.simTime;
      // if( (counter % 4) == 0)
      // {
        ROS_ERROR("Name: %s %f %f %f", name.c_str(), this->Pose1.Y() , this->lastPose1.Y() ,  obstacles_info.obstacles.at(0).velocities.twist.linear.y);
        ROS_ERROR("Name: %s %f %f %f", name.c_str(), this->Pose2.Y() , this->lastPose2.Y() ,  obstacles_info.obstacles.at(1).velocities.twist.linear.y);
        ROS_ERROR("Name: %s %f %f %f", name.c_str(), this->Pose3.Y() , this->lastPose3.Y() ,  obstacles_info.obstacles.at(2).velocities.twist.linear.y);
        ROS_ERROR("Name: %s %f %f %f", name.c_str(), this->Pose4.Y() , this->lastPose4.Y() ,  obstacles_info.obstacles.at(3).velocities.twist.linear.y);
        ROS_ERROR("$$$$$");
        pub.publish(obstacles_info);
      // }


      
    }


    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // public: static geometry_msgs::Vector3 vec3_2;
    private: std::vector<double> box_size;

    private:
      ros::NodeHandle nh;
      ros::Publisher pub;

      gazebo::common::Time lastUpdate;
      ignition::math::Pose3d lastPose1;
      ignition::math::Pose3d lastPose2;
      ignition::math::Pose3d lastPose3;
      ignition::math::Pose3d lastPose4;

      ignition::math::Pose3d Pose1;
      ignition::math::Pose3d Pose2;
      ignition::math::Pose3d Pose3;
      ignition::math::Pose3d Pose4;


      


      costmap_converter::ObstacleArrayMsg obstacles_info;
     
    
  };
  // geometry_msgs::Vector3 makeVec()
  // {
  //   geometry_msgs::Vector3 v;
  //   v.x =5; v.y = 0 ; v.z = 0;
  //   return v;
  // }
  // geometry_msgs::Vector3 AnimatedBox::vec3_2 = gazebo::makeVec();



  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

