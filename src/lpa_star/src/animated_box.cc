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
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: 
      void setBoxSizeParam(int i)
      {
        
        // gazebo::physics::BasePtr link = model->GetChild(0); //we get the link but its still the basePtr - so below we cast it 
        gazebo::physics::LinkPtr link = models.at(i)->GetLink("link");
        // gazebo::physics::CollisionPtr collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(link->GetChild(0));
        gazebo::physics::CollisionPtr collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(link->GetCollision("collision"));
        gazebo::physics::ShapePtr shape(collision->GetShape());
        gazebo::physics::BoxShapePtr box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape);
        ignition::math::Vector3d  vec3 = box->Size();
        box_size.at(i).push_back(vec3[0]);box_size.at(i).push_back(vec3[1]); box_size.at(i).push_back(vec3[2]);
        
        try{
          nh.setParam(this->models.at(i)->GetName()+"_size" ,box_size.at(i) );
        }
        catch(const std::exception& e)
        {
          std::cout<<"Exception: "<<e.what()<<"\n";
        }
      }


      void setAnimationForBoxes(int i)
      {
        ROS_ERROR("ANIMMMMMMM %i", i);
        double x = box_init_position.at(i).at(0);
        double y = box_init_position.at(i).at(1);
        gazebo::common::PoseAnimationPtr anim(
              new gazebo::common::PoseAnimation("test", 80.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(x, -y, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set waypoint location after 2 seconds
        key = anim->CreateKeyFrame(40.0);
        key->Translation(ignition::math::Vector3d(x, +y, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // set final location equal to starting location
        key = anim->CreateKeyFrame(80.0);
        key->Translation(ignition::math::Vector3d(x, -y, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));




        ROS_ERROR("ANIM %i",i);
        // set the animation
        models.at(i)->SetAnimation(anim);
      }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      int argc; char** argv;
      ros::init(argc,argv , "box_size");
      pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles",4);

      this->model = _parent; //boxes
      
      // ROS_ERROR("CHILD COUNT %i" , model->GetChildCount());
      obstacles_info.obstacles.resize(4);
      obstacles_info.obstacles.at(0).polygon.points.resize(1);
      obstacles_info.obstacles.at(0).orientation.w = 1;
      obstacles_info.obstacles.at(1).polygon.points.resize(1);
      obstacles_info.obstacles.at(1).orientation.w = 1;
      obstacles_info.obstacles.at(2).polygon.points.resize(1);
      obstacles_info.obstacles.at(2).orientation.w = 1;
      obstacles_info.obstacles.at(3).polygon.points.resize(1);
      obstacles_info.obstacles.at(3).orientation.w = 1;
      obstacles_info.header.frame_id = "odom";
      obstacles_info.obstacles.at(0).id = 1;
      obstacles_info.obstacles.at(1).id = 2;
      obstacles_info.obstacles.at(2).id = 3;
      obstacles_info.obstacles.at(3).id = 4;
      box_size.resize(model->GetChildCount());

      lastPose.resize(model->GetChildCount());
       
      box_init_position.resize(model->GetChildCount());
      box_init_position.at(0).push_back(5);box_init_position.at(0).push_back(3); lastPose.at(0).SetX(5) ; lastPose.at(0).SetY(3);
      box_init_position.at(1).push_back(10);box_init_position.at(1).push_back(-3);lastPose.at(1).SetX(10) ; lastPose.at(1).SetY(-3);
      box_init_position.at(2).push_back(15);box_init_position.at(2).push_back(3);lastPose.at(2).SetX(15) ; lastPose.at(2).SetY(3);
      box_init_position.at(3).push_back(20);box_init_position.at(3).push_back(-3);lastPose.at(3).SetX(20) ; lastPose.at(3).SetY(-3);
      
      for( int i = 0 ; i<model->GetChildCount(); i++)
      {
        models.push_back(boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetChild(i)));
        // ROS_ERROR("model %s" , models.at(i)->GetName().c_str());
        setBoxSizeParam(i);
        setAnimationForBoxes(i);
      }
      
      this->lastUpdate = gazebo::common::Time(0);


      

      https://answers.gazebosim.org//question/19883/can-someone-explain-the-role-of-connectworldupdatebegin-function/
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AnimatedBox::OnUpdate, this,std::placeholders::_1));
    
    }

    public:
      void setObstacleInfo(int i ,double dt)
      {
        gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->models.at(i)->GetLink("link"));
        ignition::math::Pose3d Pose = link->WorldCoGPose();

        obstacles_info.obstacles.at(i).velocities.twist.linear.y = (Pose.Y() - this->lastPose.at(i).Y()) / dt; // Teb uses constant velocity model but who cares :)

        double yaw = atan2(obstacles_info.obstacles.at(i).velocities.twist.linear.y  , 0);
        tf2::Quaternion q;
        q.setRPY(0,0,yaw);
        geometry_msgs::Quaternion qq; qq.w = q.getW() ; qq.x=q.getX() ; qq.y=q.getY() ; qq.z = q.getZ();
        obstacles_info.obstacles.at(i).orientation = qq;

        obstacles_info.obstacles.at(i).polygon.points[0].x = Pose.X();
        obstacles_info.obstacles.at(i).polygon.points[0].y = Pose.Y();
        obstacles_info.obstacles.at(i).polygon.points[0].z = Pose.Z();

        // ROS_ERROR("Name: %s %f %f %f", name.c_str(), Pose.Y() , this->lastPose.Y() ,  obstacles_info.obstacles.at(0).velocities.twist.linear.y);
        this->lastPose.at(i).SetY(Pose.Y());
      }

    public: void OnUpdate(const common::UpdateInfo& info) //https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot
    {
      obstacles_info.header.stamp = ros::Time::now();

      for( int i = 0 ; i<model->GetChildCount(); i++)
      {
        double dt = (info.simTime - this->lastUpdate).Double();
        setObstacleInfo(i,dt);
      }
      this->lastUpdate = info.simTime;
      pub.publish(obstacles_info);
    }


    public: 

    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        std::vector<std::vector<double>> box_size;
        std::vector<std::vector<double>> box_init_position;
        std::vector<ignition::math::Pose3d> lastPose;

        ros::NodeHandle nh;
        ros::Publisher pub;

        std::vector<physics::ModelPtr> models;
        
        gazebo::common::Time lastUpdate;
        costmap_converter::ObstacleArrayMsg obstacles_info;
     
    
  };
 



  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}