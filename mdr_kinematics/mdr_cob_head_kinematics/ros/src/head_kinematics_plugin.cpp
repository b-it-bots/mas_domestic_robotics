#include <mdr_cob_head_kinematics/head_kinematics_plugin.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <head_kinematics.h>

using namespace mdr_cob_head_kinematics;


head_kinematics_plugin::head_kinematics_plugin()
{
}


head_kinematics_plugin::~head_kinematics_plugin()
{
}


bool head_kinematics_plugin::initialize(const std::string &robot_description,
		const std::string &group_name, const std::string &base_frame,
		const std::string &tip_frame, double search_discretization)
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::searchPositionIK(
		const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::searchPositionIK(
		const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		const std::vector<double> &consistency_limits,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::searchPositionIK(
		const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		std::vector<double> &solution,
		const IKCallbackFn &solution_callback,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::searchPositionIK(
		const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		const std::vector<double> &consistency_limits,
		std::vector<double> &solution,
		const IKCallbackFn &solution_callback,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


bool head_kinematics_plugin::getPositionFK(
		const std::vector<std::string> &link_names,
		const std::vector<double> &joint_angles,
		std::vector<geometry_msgs::Pose> &poses) const
{
	ROS_ERROR("Not supported by this plugin!");

	return false;
}


const std::vector<std::string> &head_kinematics_plugin::getJointNames() const
{
	return joint_names_;
}


const std::vector<std::string> &head_kinematics_plugin::getLinkNames() const
{
	return link_names_;
}


PLUGINLIB_EXPORT_CLASS(mdr_cob_head_kinematics::head_kinematics_plugin, kinematics::KinematicsBase)
