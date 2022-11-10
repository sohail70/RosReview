#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<action_custom_msg/CustomActionAction.h>
#include<functional>

#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include<opencv4/opencv2/highgui.hpp>

#include<vector>

class TakePic
{
    public:
        TakePic(std::string action_name_):action_name(action_name_),server_(action_name_,std::bind(&TakePic::exec,this,std::placeholders::_1),false)
        {
            server_.start();
            sub = nh.subscribe("/front_cam/camera/image",1,&TakePic::takeImage,this);
            success = true;
        }

        void exec(const action_custom_msg::CustomActionGoal::ConstPtr& goal)
        {
            // if(gotFirstImage==false)
            // {
            //     ros::Duration(1).sleep();
            //     ros::spinOnce();
            // }
            int sec = goal->seconds;
            ros::Rate loop_rate(1);
            int time = 0;
            while(time<sec)
            {
                if(server_.isPreemptRequested()==true)
                {
                    server_.setPreempted();
                    success=false;
                    break;
                }
                saveImage();
                feedback_.lastPic = current_image_comp;
                server_.publishFeedback(feedback_);
                loop_rate.sleep();
                time++;
            }

            if(success)
            {
                result_.allPictures = whole_image;
                server_.setSucceeded(result_);
            }

        }

        void takeImage(const sensor_msgs::Image::ConstPtr& img)
        {
            current_image_comp.data = img->data;
            current_image = cv_bridge::toCvShare(img,"rgb8")->image;
            gotFirstImage = true;
        }

        void saveImage()
        {
            std::string soheil = "soheil" + std::to_string(counter)+".jpg";
            counter++;
            ROS_INFO("saving image: %s",soheil.c_str());
            bool b = cv::imwrite("/home/sohail/img/"+soheil,current_image);
            whole_image.push_back(current_image_comp);

        }


    private:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<action_custom_msg::CustomActionAction> server_;
        action_custom_msg::CustomActionResult result_;
        action_custom_msg::CustomActionFeedback feedback_;

        std::string action_name;
        bool success;

        ros::Subscriber sub;

        cv::Mat  current_image; // current image in openCV form
        sensor_msgs::Image current_image_comp; //current image in compressed form
        std::vector<sensor_msgs::Image> whole_image; //whole images in that n seconds
        int counter =0 ;
        bool gotFirstImage = false;
};




int main(int argc , char** argv)
{
    ros::init(argc,argv,"takePic");
    TakePic t("take_pic_action");

    ros::spin();
}