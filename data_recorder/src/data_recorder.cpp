#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"

using namespace std;

struct states{
float table_pos;
float table_vel;
cv::Mat image;

}s;




void speedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Velocity: [%f]", msg->data);
  s.table_vel = msg->data;	
}

void positionCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Position: [%f]", msg->data);
  s.table_pos = msg->data;

}

void ImgCallback(const std_msgs::Float32::ConstPtr& msg)
{
  
  s.image = cv::Mat(2,2,CV_8UC3,cv::Scalar(0,0,0));

}

int main(int argc, char **argv)
{
    string save_dir = string(argv[1]);

    ros::init(argc, argv, "data_recorder");


    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<std_msgs::Float32>("rotating_table_topic/speed", 1000, speedCallback);
    ros::Subscriber sub2 = n.subscribe<std_msgs::Float32>("rotating_table_topic/position", 1000, positionCallback);
    ros::Subscriber subImg = n.subscribe<std_msgs::Float32>("camera/image", 1000, ImgCallback);
    //ros::Subscriber subPoints = n.subscribe<std_msgs::Float32>("camera/points", 1000, PointsCallback);
    //ros::Subscriber subDepth = n.subscribe<std_msgs::Float32>("camera/depth", 1000, DepthCallback);
    // ros::Publisher speedPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/command/speed",1000);
    ros::Publisher posPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/command/position",1000);


    ros::Rate loop_rate(1);


    while (ros::ok())
    {
        std_msgs::Float32 speed;
        std_msgs::Float32 position;

        for(float a=0.0;a<360.0;a+=1.0){
          while(s.table_pos!=a){
            position.data = a;
	    posPublischer.publish(position);
          }
	  if(s.table_pos==a){
	     loop_rate.sleep();//wait 1s to make sure the image is updated
	     string fullname = save_dir + "/" + std::to_string(static_cast<int>(a)) + ".png";
 	     cout<<fullname<<endl;
	     cv::imwrite(fullname,s.image);
	  }
        }

        //ros::spin();
        
    }




    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
//    ros::spin();

    return 0;
}
