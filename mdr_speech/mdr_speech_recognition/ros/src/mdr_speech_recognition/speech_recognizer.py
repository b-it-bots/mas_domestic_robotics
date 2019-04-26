#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import httplib
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognizer(object):

    def __init__(self):
        rospy.init_node("speech_recognizer")
        self.pub = rospy.Publisher("speech_recognizer", String, latch=True, queue_size=1)
        self.model_directory = rospy.get_param('~model_directory')
        self.use_kaldi = rospy.get_param('~use_kaldi')
        self.recognizer = sr.Recognizer()
        if self.use_kaldi:
            try:
                self.recognizer.load_kaldi_model(model_directory=self.model_directory)
            except:
                self.use_kaldi = False
                rospy.logerr(sys.exc_info()[0])
                rospy.logerr('Unable to load Kaldi model. Using PocketSphinx as offline speech recognition')
        self.microphone = sr.Microphone()

    @staticmethod
    def check_internet_connection():
        connection = httplib.HTTPConnection("www.google.com", timeout=5)
        try:
            connection.request("HEAD", "/")
            connection.close()
            return True
        except:
            connection.close()
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
                        if self.use_kaldi:
                            recognized_speech = self.recognizer.recognize_kaldi(audio)[0]
                        else:
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
    speech_recognizer = SpeechRecognizer()
    speech_recognizer.recognize()
    rospy.spin()
