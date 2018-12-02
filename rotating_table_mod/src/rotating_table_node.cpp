#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "stepper.hpp"

stepper *myStepper;




void speedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Velocity to set: [%f]", msg->data);

  //myStepper->setVelocity(msg->data);
//  myStepper->setVelocityWithRamp(msg->data);
  myStepper->setVelocityWithRampThreaded(msg->data);

}

void positionCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Position to set: [%f]", msg->data);

//  myStepper->setPosition(msg->data);
  myStepper->setPositionThreaded(msg->data);

}

int main(int argc, char **argv)
{

    myStepper=new stepper();
    myStepper->startLoop();

    ros::init(argc, argv, "rotating_table_node");


    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<std_msgs::Float32>("rotating_table_topic/command/speed", 1000, speedCallback);
    ros::Subscriber sub2 = n.subscribe<std_msgs::Float32>("rotating_table_topic/command/position", 1000, positionCallback);
    ros::Publisher speedPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/speed",1000);
    ros::Publisher posPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/position",1000);


    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        std_msgs::Float32 speed;
        std_msgs::Float32 position;

        myStepper->getVelocity(speed.data);
        myStepper->getPosition(position.data);

        speedPublischer.publish(speed);
        posPublischer.publish(position);

        ros::spinOnce();
        loop_rate.sleep();
    }




    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
//    ros::spin();

    return 0;
}
