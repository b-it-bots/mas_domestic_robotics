#include <ros/ros.h>
#include <mcr_behaviours_msgs/CleanTable.h>

std::string g_strNodeName = "brsu_table_cleaning_sim";


bool clean(mcr_behaviours_msgs::CleanTable::Request &req, mcr_behaviours_msgs::CleanTable::Response &res) {
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, g_strNodeName);
        ros::NodeHandle nh("~");

	ros::ServiceServer srv_start = nh.advertiseService("table_cleaning", clean);

	ros::spin();
	
	return 0;
}
