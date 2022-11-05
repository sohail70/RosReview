#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<actionlib/TestAction.h>
#include<geometry_msgs/Twist.h>
#include<functional>
#define PI 3.14
class MoveSquareAction{
    public:
        MoveSquareAction(std::string name_): action_name(name_) , as_(nh_,name_ , std::bind(&MoveSquareAction::exec , this,std::placeholders::_1) , false)
        {
            as_.start();
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
        }


        void exec(const actionlib::TestGoal::ConstPtr& goal)
        {
            side = goal->goal;
            double x_speed = 1;
            double z_speed = 1;
            double w_z = .1;
            double height = 1;

            take_off(z_speed, height);
            success = true;
            for(int i = 0 ; i < 4 ; i++)
            {
                if(as_.isPreemptRequested() || !ros::ok())
                {
                    as_.setPreempted();
                    success = false;
                    break;
                }

                //squaring 
                move_forward(x_speed);
                stop();
                turn(w_z);
                stop();
                feedback_.feedback = i;
                as_.publishFeedback(feedback_);
            }

            if(success)
            {
                result_.result = 1;
                ROS_INFO("Result is: %i" , result_.result);
                ROS_INFO("%s: succeeded",action_name.c_str() );
                as_.setSucceeded(result_);
                this->stop();
                this->land();
            }


        }

        void take_off(double z_speed , double height)
        {
            velocity.linear.z = z_speed;
            vel_pub.publish(velocity);
            double time = height/z_speed;
            ros::Duration(time).sleep();
            velocity.linear.z = 0;
            vel_pub.publish(velocity);
        }

        void move_forward(double x_speed)
        {
            velocity.linear.x = x_speed;
            vel_pub.publish(velocity);
            double time = side /x_speed;
            ros::Duration(time).sleep();
        }

        void turn(double w_z)
        {
            velocity.angular.z = w_z;
            vel_pub.publish(velocity);
            double time = (PI/2) / w_z;
            ros::Duration(time).sleep();
        }

        void land()
        {
            velocity.linear.z = -1;
            vel_pub.publish(velocity);
            ros::Duration(6).sleep();
            velocity.linear.z = 0;
            vel_pub.publish(velocity);
        }

        void stop()
        {
            velocity.linear.x = 0;
            velocity.linear.z = 0;
            vel_pub.publish(velocity);
        }

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<actionlib::TestAction> as_;
        actionlib::TestResult result_;
        actionlib::TestFeedback feedback_;

        std::string action_name;

        ros::Publisher vel_pub;
        geometry_msgs::Twist velocity;
        bool success;
        double side;
};

int main(int argc , char** argv)
{
    ros::init(argc,argv, "square_action");
    MoveSquareAction square("soheil_action");
    ros::spin();
    return 0;
}