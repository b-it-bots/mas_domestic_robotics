/*
 * door_status_node.cpp
 *
 *  Created on: Jan 26, 2011
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>

bool bIsDoorOpen = false;

ros::Publisher pubDoorStatus;
ros::Subscriber subChangeDoorStatus;

std_msgs::Bool bMsg;

void publishDoorStatus()
{
    bMsg.data = bIsDoorOpen;
    pubDoorStatus.publish(bMsg);
}

void changeDoorStatus(const std_msgs::BoolConstPtr status)
{
    bIsDoorOpen = status->data;

    if (bIsDoorOpen)
    {
        ROS_INFO("Door status: door is OPEN");
    }
    else
    {
        ROS_INFO("Door status: door is CLOSED");
    }
    publishDoorStatus();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_status_mockup");
    ros::NodeHandle nh("~");


    pubDoorStatus = nh.advertise<std_msgs::Bool>("door_status", 1, true);

    subChangeDoorStatus = nh.subscribe<std_msgs::Bool>("change_door_status", 1, changeDoorStatus);

    ros::Rate loop_rate(10);

    publishDoorStatus();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
