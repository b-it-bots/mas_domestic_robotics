#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cob_msgs/EmergencyStopState.h>
#include <string>
#include <std_msgs/String.h>

cob_msgs::EmergencyStopState last_emergency_stop_state;
ros::Publisher say_client;

void emergencyCallback(const cob_msgs::EmergencyStopState& data)
{
	std_msgs::String message;

	if (last_emergency_stop_state.emergency_button_stop
			!= data.emergency_button_stop) {
		if (data.emergency_button_stop == true){
			message.data = "emergency button pressed";
			ROS_INFO("%s", message.data.c_str());
			say_client.publish(message);
		} else {
			message.data = "emergency button released";
			ROS_INFO("%s", message.data.c_str());
			say_client.publish(message);
		}
	}

	if (last_emergency_stop_state.scanner_stop != data.scanner_stop) {
		if (data.scanner_stop == true) {
			message.data = "laser scanner emergency issued";
			ROS_INFO("%s", message.data.c_str());
			say_client.publish(message);
		} else {
			message.data = "laser scanner emergency released";
			ROS_INFO("%s", message.data.c_str());
			say_client.publish(message);
		}
	}

	last_emergency_stop_state = data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mdr_emergency_stop_monitor");
	ros::NodeHandle n("~");
 	
	say_client = n.advertise<std_msgs::String>("say", 1, true);
	ros::Subscriber emergency_sub = n.subscribe("/emergency_stop_state", 10, emergencyCallback);


	ros::spin();

	return 0;
}

