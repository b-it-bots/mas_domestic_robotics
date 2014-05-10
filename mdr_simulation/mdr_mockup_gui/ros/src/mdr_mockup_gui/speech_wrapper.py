#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from mcr_speech_msgs.msg import RecognizedSpeech

class MockupSpeechWrapper:


    def request_cb(self, string_msgs):
        rospy.loginfo("request: " + str(string_msgs))
        say_msgs = String()
        say_msgs.data = str(string_msgs)

        self.new_speech_arrived = False
        
        self.say_publisher.publish(say_msgs)

        now = rospy.get_rostime()

        while((self.new_speech_arrived == False) and (now + rospy.Duration(30)) > rospy.get_rostime()):
            rospy.logerr("waiting....")
            rospy.sleep(0.05)

        if (self.new_speech_arrived):
            reply_msgs = String()
            reply_msgs.data = self.glrs_response.understood_phrase
            self.reply.publish(reply_msgs)
        else:
            rospy.logerr("No response on regonized_speech topic")

    def glrs_cb(self, msg):
        rospy.loginfo("Received recognized speech " + str(msg.understood_phrase))
        self.glrs_response = msg
        self.new_speech_arrived = True
        # TODO IMPLEMENT

    def init(self):
        rospy.logdebug("creating publisher and subscriber")
        self.reply = rospy.Publisher('~reply', String)
        self.callback = rospy.Subscriber("~request", String, self.request_cb)

        self.say_publisher = rospy.Publisher('~say', String)

        self.glrs_cb = rospy.Subscriber('~recognize_speech', RecognizedSpeech, self.glrs_cb)
 
        rospy.loginfo("Done initializing")

def main():
        rospy.init_node('mdr_mockup_speech_wrapper')
        
        wrapper = MockupSpeechWrapper()
        wrapper.init()

        rospy.spin()
