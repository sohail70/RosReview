#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include<string>
#include<lpa_star/dynamic_obs.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      int argc; char** argv;
      ros::init(argc,argv , "box_size");
      this->model = _parent;


     

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
      }
      else if (_parent->GetName()=="box_2")
      {
        x=10;y=-5;
      }
      else if(_parent->GetName()=="box_3")
      {
        x=15;y=5;
      }
      else if(_parent->GetName()=="box_4")
      {
        x=20;y=-5;
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

      // set the animation
      _parent->SetAnimation(anim);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AnimatedBox::OnUpdate, this));
    
    }
    public: void OnUpdate()
    {
      // ROS_WARN("This function is called every time! I don't need it now");
    }


    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // public: static geometry_msgs::Vector3 vec3_2;
    private: std::vector<double> box_size;

    private:
      ros::NodeHandle nh;
     
    
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

