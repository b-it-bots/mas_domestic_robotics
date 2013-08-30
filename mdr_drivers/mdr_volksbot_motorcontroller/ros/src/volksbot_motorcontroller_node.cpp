/*
 * volksbot_motorcontroller_node.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <pr2_msgs/PowerState.h>
#include <tf/tf.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "robot_factory.h"

#define COVARIANCE_SIZE		36

double base_linear_speed = 0;
double base_angular_speed = 0;

void base_commands(const geometry_msgs::Twist::ConstPtr& command)
{
	base_linear_speed = command->linear.x;
	base_angular_speed = command->angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mdr_volksbot_motorcontroller");
	ros::NodeHandle nh("~");

	// Publisher and Subscriber
	ros::Subscriber sub_base = nh.subscribe < geometry_msgs::Twist > ("cmd_vel", 1, base_commands);
	ros::Publisher pub_odom = nh.advertise < nav_msgs::Odometry > ("odom", 1);
	ros::Publisher pub_battery = nh.advertise < pr2_msgs::PowerState > ("battery_status", 1);
	ros::Publisher pub_joints = nh.advertise < sensor_msgs::JointState > ("joint_states", 1);

	tf::TransformBroadcaster tf_broadcast_odometry_;	// according transformation for the tf broadcaster

	ros::Time battery_timer = ros::Time::now();
	ros::Duration time_diff;

	sensor_msgs::JointState joint_state;
	nav_msgs::Odometry odometry_msg;
	pr2_msgs::PowerState battery_state;
	int battery_level = 0;
	bool is_charging = false;

	double covariance[COVARIANCE_SIZE] =
	{ 1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3 };

	CRobotFactory *robot_factory = new CRobotFactory();
	CRobotPlatformDifferentialDrive *differential_drive = NULL;

	// get parameters
	string device_name = "/dev/volksbot/vb_motorcontroller";
	if (nh.getParam("device_name", device_name) == false)
		ROS_WARN("Parameter device_name not specified in launch file, used default value: %s", device_name.c_str());

	// establish connection to the motorcontroller
	if (!robot_factory->connect(device_name))
	{
		ROS_ERROR_STREAM("Could not connect to the VolksBot motor controller on " << device_name);
		return -1;
	}

	differential_drive = robot_factory->getRobotPlatformDifferentialDrive();
	unsigned int tics_per_turn_of_wheel_left = differential_drive->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelLeft;
	unsigned int tics_per_turn_of_wheel_right = differential_drive->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelLeft;

	double radian_per_tic_left = (2.0 * M_PI) / (double) tics_per_turn_of_wheel_left;
	double radian_per_tic_right = (2.0 * M_PI) / (double) tics_per_turn_of_wheel_right;
	int encoder_tics_left = 0, encoder_tics_right = 0;

	ROS_INFO("Successfully connected to VolksBot motorcontroller");

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		// ############################################################
		// set velocities
		// ############################################################
		differential_drive->move(base_linear_speed, base_angular_speed);

		// ############################################################
		// get and publish battery state
		// ############################################################
		time_diff = ros::Time::now() - battery_timer;
		if (time_diff.toSec() > 5)	// publish every 5 seconds
		{
			robot_factory->getBatteryStatus(battery_level, is_charging);

			battery_state.relative_capacity = battery_level;
			battery_state.AC_present = is_charging;

			pub_battery.publish(battery_state);
			battery_timer = ros::Time::now();
		}

		// ############################################################
		// get and publish odometry information
		// ############################################################
		odometry_msg.header.stamp = ros::Time::now();
		odometry_msg.header.frame_id = "/odom";
		odometry_msg.child_frame_id = "base_link";
		odometry_msg.pose.pose.position.x = differential_drive->getPose()->dX / 1000;
		odometry_msg.pose.pose.position.y = differential_drive->getPose()->dY / 1000;
		odometry_msg.pose.pose.position.z = 0.0;
		odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(differential_drive->getPose()->dYaw);
		odometry_msg.twist.twist.linear.x = differential_drive->getPoseSihft()->dX / 1000;  // vx
		odometry_msg.twist.twist.linear.y = differential_drive->getPoseSihft()->dY / 1000;  // vy
		odometry_msg.twist.twist.angular.z = differential_drive->getPoseSihft()->dYaw;  // vtheta

		for (int i = 0; i < COVARIANCE_SIZE; ++i)
		{
			odometry_msg.pose.covariance[i] = covariance[i];
			odometry_msg.twist.covariance[i] = covariance[i];
		}

		if (base_linear_speed == 0 && base_angular_speed == 0)
		{
			odometry_msg.pose.covariance[0] = 1e-9;
			odometry_msg.pose.covariance[7] = 1e-9;
			odometry_msg.pose.covariance[35] = 1e-9;
			odometry_msg.twist.covariance[0] = 1e-9;
			odometry_msg.twist.covariance[7] = 1e-9;
			odometry_msg.twist.covariance[35] = 1e-9;
		}

		pub_odom.publish(odometry_msg);

		// ############################################################
		//publish real front wheel positions and fake castor positions
		// ############################################################
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);

		// get encoder tics
		differential_drive->retrieveAbsoluteEncoderTics(encoder_tics_left, encoder_tics_right);

		//real values
		joint_state.name[0] = "base_link_l_front_wheel_joint";
		if (encoder_tics_left >= 0)
			joint_state.position[0] = (double) (encoder_tics_left % tics_per_turn_of_wheel_left) * radian_per_tic_left;
		else
			joint_state.position[0] = -(double) (abs(encoder_tics_left) % tics_per_turn_of_wheel_left) * radian_per_tic_left;

		joint_state.name[1] = "base_link_r_front_wheel_joint";
		if (encoder_tics_right >= 0)
			joint_state.position[1] = (double) (encoder_tics_right % tics_per_turn_of_wheel_right) * radian_per_tic_right;
		else
			joint_state.position[1] = -(double) (abs(encoder_tics_right) % tics_per_turn_of_wheel_right) * radian_per_tic_right;

		//fake values
		joint_state.name[2] = "base_link_l_rear_wheel_joint";
		joint_state.position[2] = 0;
		joint_state.name[3] = "base_link_r_rear_wheel_joint";
		joint_state.position[3] = 0;
		joint_state.name[4] = "l_rotation_joint";
		joint_state.position[4] = 0;
		joint_state.name[5] = "r_rotation_joint";
		joint_state.position[5] = 0;

		pub_joints.publish(joint_state);

		// compose and publish transform for tf package
		geometry_msgs::TransformStamped odom_tf;
		// compose header
		odom_tf.header.stamp = ros::Time::now();
		odom_tf.header.frame_id = "/odom";
		odom_tf.child_frame_id = "/base_footprint";
		// compose data container
		odom_tf.transform.translation.x = odometry_msg.pose.pose.position.x;
		odom_tf.transform.translation.y = odometry_msg.pose.pose.position.y;
		odom_tf.transform.translation.z = odometry_msg.pose.pose.position.z;
		odom_tf.transform.rotation = odometry_msg.pose.pose.orientation;

		tf_broadcast_odometry_.sendTransform(odom_tf);

		ros::spinOnce();
		loop_rate.sleep();
	}

}

