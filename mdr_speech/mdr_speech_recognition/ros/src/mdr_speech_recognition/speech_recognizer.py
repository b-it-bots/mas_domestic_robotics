#!/usr/bin/env python
from __future__ import print_function
import rospy
import urllib2
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognizer:

    def __init__(self):
        self.pub = rospy.Publisher("speech_recognizer", String, latch=True, queue_size=1)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    """
    Currently 172.217.21.238 is one of IP addresses of google.com.
    It might happen, that this IP expires. In this case it has to be changed manually.
    Use the following command to find a current IP address for google.com:
        $ dig google.com  +trace
    """
    @staticmethod
    def check_internet_connection():
        try:
            urllib2.urlopen("http://172.217.21.238", timeout=1)
            return True
        except urllib2.URLError:
            return False

    def recognize(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        try:
            while not rospy.is_shutdown():
                rospy.loginfo('Listening...')
                with self.microphone as source:
                    audio = self.recognizer.listen(source)
                rospy.loginfo('Got a sound; recognizing...')

                """
                Google over PocketSphinx: In case there is a internet connection
                use google, otherwise use pocketsphinx for speech recognition.
                """
                recognized_speech = ""
                if SpeechRecognizer.check_internet_connection():
                    try:
                        recognized_speech = self.recognizer.recognize_google(audio)
                    except sr.UnknownValueError:
                        rospy.logerr("Could not understand audio.")
                    except sr.RequestError:
                        rospy.logerr("Could not request results.")
                else:
                    try:
                        recognized_speech = self.recognizer.recognize_sphinx(audio)
                    except sr.UnknownValueError:
                        rospy.logerr("Could not understand audio.")
                    except sr.RequestError:
                        rospy.logerr("Could not request results.")

                if recognized_speech != "":
                    rospy.loginfo("You said: " + recognized_speech)
                    self.pub.publish(recognized_speech)

        except Exception as exc:
            rospy.logerr(exc)

def main():
    rospy.init_node("speech_recognizer")
    speech_recognizer = SpeechRecognizer()
    speech_recognizer.recognize()
    rospy.spin()
