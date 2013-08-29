#!/usr/bin/env python
import rospy

#from mdr_monitoring_msgs.msg import AudioEndpoint
from mdr_monitoring_msgs.msg import Endpoints

from std_msgs.msg import ColorRGBA

light_pub = rospy.Publisher('light_controller/command', ColorRGBA, latch = True)

red_light = ColorRGBA()
red_light.r = 1.0
red_light.g = 0.0
red_light.b = 0.0
red_light.a = 1.0

green_light = ColorRGBA()
green_light.r = 0.0
green_light.g = 1.0
green_light.b = 0.0
green_light.a = 1.0


def callback(endpoints):
	rospy.logdebug(rospy.get_name()+ ": %s" % endpoints)

	if len(endpoints.endpoints) == 0:
		light_pub.publish(green_light)
		rospy.logdebug('Mic: All is fine!')
	else:
		light_pub.publish(red_light)
		rospy.logdebug('Mic: Broken!')
	
		
def get_audio_device_state():
	rospy.Subscriber("/audio_status", Endpoints, callback)
	rospy.spin()
	
	
def main():
	rospy.init_node('audio_monitor')
	get_audio_device_state()
