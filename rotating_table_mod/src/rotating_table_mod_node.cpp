#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "steppermodified.hpp"

stepperdir *myStepperDir;




void speedCallbackDir(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Velocity to set: [%f]", msg->data);

  //myStepper->setVelocity(msg->data);
//  myStepper->setVelocityWithRamp(msg->data);
  myStepperDir->setVelocityWithRampThreaded(msg->data);

}

void positionCallbackDir(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Position to set: [%f]", msg->data);

//  myStepper->setPosition(msg->data);
  myStepperDir->setPositionThreadedWithDir(msg->data,-1.0);

}

int main(int argc, char **argv)
{

    myStepperDir=new stepperdir();
    myStepperDir->startLoop();

    ros::init(argc, argv, "rotating_table_node");
    
    float timer = 30.0;
    float startDir = -1.0;
    ros::NodeHandle n;

   // ros::Subscriber sub = n.subscribe<std_msgs::Float32>("rotating_table_topic/command/speed", 1000, speedCallback);
  //  ros::Subscriber sub2 = n.subscribe<std_msgs::Float32>("rotating_table_topic/command/position", 1000, positionCallback);
    ros::Publisher speedPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/speed",1000);
    ros::Publisher posPublischer = n.advertise<std_msgs::Float32>("rotating_table_topic/position",1000);
    ros::Publisher dirPublisher = n.advertise<std_msgs::Float32>("rotating_table_topic/direction",1000);

    std_msgs::Float32 position;
    std_msgs::Float32 direction;
    myStepperDir->getPosition(position.data);
    ros::Rate loop_rate(10);
    float pos = position.data;

//Programm wait 30 sek -> move 180 deg in start dir -> wait 30 sek -> move 180 deg in start dir -> change dir and restart
    while (ros::ok())
    {
        /*current pos is start pos*/
        //std_msgs::Float32 speed;


        //myStepper->getVelocity(speed.data);
        direction.data = startDir;
        ROS_INFO("richtung: %f", startDir);
        ROS_INFO("position: %f", position.data);

        
        posPublischer.publish(position);
        dirPublisher.publish(direction);
        ros::Duration(timer).sleep(); //sleep for the time
        //speedPublischer.publish(speed);

        pos = pos+180.0*(170.0/12.0)*startDir;
        ROS_INFO("pos: %f", pos);
        myStepperDir->setPositionThreadedWithDir(pos,startDir);//move for 180 degree
        ros::Duration(timer).sleep(); //sleep for the time
        position.data = pos;
        posPublischer.publish(position);

        pos = pos+180.0*(170.0/12.0)*startDir;
	    ROS_INFO("pos: %f", pos);
        myStepperDir->setPositionThreadedWithDir(pos,startDir);//move for 180 degree
        position.data = pos;
        posPublischer.publish(position);
        startDir = startDir*(-1.0);
        direction.data = startDir;
        dirPublisher.publish(direction);

        //ros::spinOnce();
        //loop_rate.sleep();
    }




    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
//    ros::spin();

    return 0;
}
