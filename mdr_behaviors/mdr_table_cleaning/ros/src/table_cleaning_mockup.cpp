#include <ros/ros.h>
#include <mcr_behaviours_msgs/CleanTable.h>

bool clean(mcr_behaviours_msgs::CleanTable::Request &req, mcr_behaviours_msgs::CleanTable::Response &res) {
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, table_cleaning_sim);
  ros::NodeHandle nh("~");

	ros::ServiceServer srv_start = nh.advertiseService("table_cleaning", clean);

	ros::spin();
	
	return 0;
}
