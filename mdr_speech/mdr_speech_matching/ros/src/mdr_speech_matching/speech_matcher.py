#!/usr/bin/env python
from __future__ import print_function
import rospy
from speech_matching.SpeechMatching import SpeechMatching
from std_msgs.msg import String
from mdr_speech_matching.msg import MatchedSentence

class SpeechMatcher:

    def __init__(self):
        self.sm = SpeechMatching()
        self.pub = rospy.Publisher("speech_matcher", MatchedSentence, latch=True, queue_size=1)
        self.result = MatchedSentence()

    def match(self, data):
        input_sentence = str(data).strip()
        matching_result = self.sm.match_sentence(input_sentence)

        type = matching_result[0]
        if type == "nothing":
            rospy.logerr("No match found.")
            return
        elif type == "question":
            self.result.type = MatchedSentence.TYPE_QUESTION
        elif type == "command":
            self.result.type = MatchedSentence.TYPE_COMMAND

        self.result.matched_sentence = matching_result[1][0]
        self.result.similarity = matching_result[1][1]

        rospy.loginfo("Received string: '" + self.result.matched_sentence  + "' with a distance of: " + str(self.result.similarity) + ".")
        self.pub.publish(self.result)


def main():
    rospy.init_node("speech_matcher")
    speech_matcher = SpeechMatcher()
    sub = rospy.Subscriber("speech_recognizer", String, speech_matcher.match)
    rospy.spin()
