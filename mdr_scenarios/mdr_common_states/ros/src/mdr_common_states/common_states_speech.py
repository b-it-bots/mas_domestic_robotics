#!/usr/bin/python

######################### IMPORTS #########################
import rospy
import smach
import smach_ros
import std_msgs.msg

class say_state(smach.State):
    def __init__(self, phrase_to_say):
        smach.State.__init__(self, outcomes=['success'])
        self.phrase_to_say = phrase_to_say
        self.speak_pub = rospy.Publisher('/sound/say', std_msgs.msg.String, latch=True)

    def execute(self, userdata):
        self.speak_pub.publish(std_msgs.msg.String(self.phrase_to_say))
        return 'success'

class say_state_dynamic(smach.State):
    prefix = ""
    suffix = ""
    def __init__(self, prefix, suffix):
        smach.State.__init__(self, outcomes=['success'], input_keys=['phrase_in'])	
        self.prefix = prefix
        self.suffix = suffix
        self.speak_pub = rospy.Publisher('/sound/say', std_msgs.msg.String, latch=True)

    def execute(self, userdata):
        self.speak_pub.publish(phrase)
        return 'success'
