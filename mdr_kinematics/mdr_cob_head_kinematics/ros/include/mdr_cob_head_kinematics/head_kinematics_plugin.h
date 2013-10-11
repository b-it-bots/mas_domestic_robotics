#ifndef HEAD_KINEMATICS_PLUGIN_H
#define HEAD_KINEMATICS_PLUGIN_H

#include <moveit/kinematics_base/kinematics_base.h>

namespace mdr_cob_head_kinematics
{

class head_kinematics_plugin : public kinematics::KinematicsBase
{
	public:
		/**
		 * Ctor.
		 */
		head_kinematics_plugin();

		/**
		 * Dtor.
		 */
		virtual ~head_kinematics_plugin();

		/**
		 * @see kinematics::KinematicsBase::initialize
		 */
		bool initialize(const std::string &robot_description,
				const std::string &group_name, const std::string &base_frame,
				const std::string &tip_frame, double search_discretization);

		/**
		 * @see kinematics::KinematicsBase::getPositionIK
		 */
		bool getPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options
						= kinematics::KinematicsQueryOptions()) const;

		/**
		 * @see kinematics::KinematicsBase::searchPositionIK
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state, double timeout,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options
						= kinematics::KinematicsQueryOptions()) const;

		/**
		 * @see kinematics::KinematicsBase::searchPositionIK
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state, double timeout,
				const std::vector<double> &consistency_limits,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options
						= kinematics::KinematicsQueryOptions()) const;

		/**
		 * @see kinematics::KinematicsBase::searchPositionIK
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state, double timeout,
				std::vector<double> &solution,
				const IKCallbackFn &solution_callback,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options
						= kinematics::KinematicsQueryOptions()) const;

		/**
		 * @see kinematics::KinematicsBase::searchPositionIK
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state, double timeout,
				const std::vector<double> &consistency_limits,
				std::vector<double> &solution,
				const IKCallbackFn &solution_callback,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options
						= kinematics::KinematicsQueryOptions()) const;

		/**
		 * @see kinematics::KinematicsBase::getPositionFK
		 */
		bool getPositionFK(const std::vector<std::string> &link_names,
				const std::vector<double> &joint_angles,
				std::vector<geometry_msgs::Pose> &poses) const;

		/**
		 * @see kinematics::KinematicsBase::getJointNames
		 */
		const std::vector<std::string> &getJointNames() const;

		/**
		 * @see kinematics::KinematicsBase::getLinkNames
		 */
		const std::vector<std::string> &getLinkNames() const;


	private:
		/**
		 * Copy ctor.
		 */
		head_kinematics_plugin(const head_kinematics_plugin &other);

		/**
		 * Assignment operator.
		 */
		head_kinematics_plugin &operator=(const head_kinematics_plugin &other);


	private:
		/**
		 * Joint names.
		 */
		std::vector<std::string> joint_names_;

		/**
		 * Link names.
		 */
		std::vector<std::string> link_names_;
};

}

#endif
