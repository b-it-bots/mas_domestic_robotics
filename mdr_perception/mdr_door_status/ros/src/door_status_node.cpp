/*
 * door_status_node.cpp
 *
 *  Created on: Jan 26, 2011
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

sensor_msgs::LaserScanConstPtr g_pLaserScanFront;
bool bIsDoorOpen = false;
double dOpeningAngle = 10.0;
double dDistanceThreshold = 1.0;

bool publish_door_state()
{
    double dAngle = 0.0;
    double dSummedDistance = 0.0;
    unsigned int dCountInAngleRange = 0;

    if (g_pLaserScanFront == 0)
        return false;


    //sum up distance in the desired opening angle
    dAngle = g_pLaserScanFront->angle_min;
    for (unsigned int i = 0; i < g_pLaserScanFront->ranges.size(); ++i, dAngle += g_pLaserScanFront->angle_increment)
    {
        //check angle against desired range and only take those distance measures
        if ((dAngle >= -dOpeningAngle) && (dAngle <= dOpeningAngle))
        {
            dSummedDistance += g_pLaserScanFront->ranges[i];
            ++dCountInAngleRange;
        }
    }

    //calc mean
    dSummedDistance /= dCountInAngleRange;

    //check mean distance to decide weather door is open or not
    if (dSummedDistance > dDistanceThreshold)
    {
        ROS_INFO("door status: door is OPEN");
        return true;
    }
    else
    {
        ROS_INFO("door status: door is CLOSED");
        return false;
    }

    return true;
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
{
    g_pLaserScanFront = msg_in;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_status");
    ros::NodeHandle nh("~");

    ros::Subscriber subFrontScan = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);
    ros::Publisher pubDoorStatus = nh.advertise<std_msgs::Bool>("door_status", 1, true);

    // read params from parameter server file
    if (nh.getParam("angular_range", dOpeningAngle) == false)
        ROS_WARN("Parameter angular_range not specified in launch file, used default value: %lf degree", dOpeningAngle);
    if (nh.getParam("distance_threshold", dDistanceThreshold) == false)
        ROS_WARN("Parameter distance_threshold not specified in launch file, used default value: %lf meter", dDistanceThreshold);

    // divide degree by 2 (to have range -angle to angle+) and convert radians
    dOpeningAngle = ((dOpeningAngle / 2.0) / 180.0) * M_PI;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        bool open = publish_door_state();
        std_msgs::Bool open_msgs;
        open_msgs.data = open;

        pubDoorStatus.publish(open_msgs);
        loop_rate.sleep();
    }

    return 0;
}
