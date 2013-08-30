#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cob_relayboard/EmergencyStopState.h"
#include "mcr_speech_msgs/Say.h"
#include <string>


cob_relayboard::EmergencyStopState lastEmergencyStopState;
ros::Publisher say_client;

void emergencyCallback(const cob_relayboard::EmergencyStopState& data){
	mcr_speech_msgs::Say message;
	
	if(lastEmergencyStopState.emergency_button_stop != data.emergency_button_stop){
		
		if(data.emergency_button_stop == true){
			message.phrase = "emergency button pressed";
			ROS_INFO(message.phrase.c_str());
			say_client.publish(message);
		}else{
			message.phrase = "emergency button released";
			ROS_INFO(message.phrase.c_str());
			say_client.publish(message);
		}
	}
	
	if(lastEmergencyStopState.scanner_stop != data.scanner_stop){
		if(data.scanner_stop == true){
			message.phrase = "laser scanner emergency issued";
			ROS_INFO(message.phrase.c_str());
			say_client.publish(message);
		}else{
			message.phrase = "laser scanner emergency released";
			ROS_INFO(message.phrase.c_str());
			say_client.publish(message);
		}
	}
	
	lastEmergencyStopState = data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "emergency_stop_monitor");
  ros::NodeHandle n;
 	
 
  say_client = n.advertise<mcr_speech_msgs::Say>("~say",1,true);
  ros::Subscriber emergency_sub = n.subscribe("/emergency_stop_state", 1000, emergencyCallback);
  
  
  ros::spin();

  return 0;
}

