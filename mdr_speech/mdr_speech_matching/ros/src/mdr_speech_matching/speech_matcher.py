#!/usr/bin/env python
from __future__ import print_function
import rospy
from speech_matching.SpeechMatching import SpeechMatching
from std_msgs.msg import String

class SpeechMatcher:

    def __init__(self):
        self.sm = SpeechMatching()

    def match(self, data):
        input_sentence = str(data).strip()
        matching_result = self.sm.match_sentence(input_sentence)
        type = matching_result[0]
        matching_sentence = matching_result[1][0]
        levenshtein_distance = matching_result[1][1]
        rospy.loginfo("Received string: '" + matching_sentence + "' with a distance of: " + str(levenshtein_distance) + ".")

def main():
    rospy.init_node("speech_matcher")
    speech_matcher = SpeechMatcher()
    sub = rospy.Subscriber("speech_recognizer", String, speech_matcher.match)
    rospy.spin()
