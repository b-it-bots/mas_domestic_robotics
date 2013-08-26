#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cob_relayboard/EmergencyStopState.h"
#include "mcr_speech_msgs/Say.h"
#include <string>


cob_relayboard::EmergencyStopState lastEmergencyStopState;
ros::ServiceClient say_client;

void emergencyCallback(const cob_relayboard::EmergencyStopState& data){
	mcr_speech_msgs::Say message;
	
	if(lastEmergencyStopState.emergency_button_stop != data.emergency_button_stop){
		
		if(data.emergency_button_stop == true){
			message.request.phrase = "emergency button pressed";
			ROS_INFO(message.request.phrase.c_str());
			say_client.call(message);
		}else{
			message.request.phrase = "emergency button released";
			ROS_INFO(message.request.phrase.c_str());
			say_client.call(message);
		}
	}
	
	if(lastEmergencyStopState.scanner_stop != data.scanner_stop){
		if(data.scanner_stop == true){
			message.request.phrase = "laser scanner emergency issued";
			ROS_INFO(message.request.phrase.c_str());
			say_client.call(message);
		}else{
			message.request.phrase = "laser scanner emergency released";
			ROS_INFO(message.request.phrase.c_str());
			say_client.call(message);
		}
	}
	
	lastEmergencyStopState = data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "brsu_monitoring");
  ros::NodeHandle n;
 	
 
  say_client = n.serviceClient<mcr_speech_msgs::Say>("/say");
  ros::Subscriber emergency_sub = n.subscribe("/emergency_stop_state", 1000, emergencyCallback);
  
  
  ros::spin();

  return 0;
}

