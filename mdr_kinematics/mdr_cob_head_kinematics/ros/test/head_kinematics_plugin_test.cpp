#include <mdr_cob_head_kinematics/head_kinematics_plugin.h>
#include <pluginlib/class_loader.h>
#include <gtest/gtest.h>


TEST(test, test1)
{
	pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader("moveit_core", "kinematics::KinematicsBase");

	try {
		ROS_ERROR("Available: %i", kinematics_loader.isClassLoaded("mdr_cob_head_kinematics::head_kinematics_plugin"));
		boost::shared_ptr<kinematics::KinematicsBase> kin = kinematics_loader.createInstance("mdr_cob_head_kinematics::head_kinematics_plugin");
	} catch(pluginlib::PluginlibException &ex) {
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

	EXPECT_TRUE(false);
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
