#!/usr/bin/env python
from __future__ import print_function
import rospy
import os
from std_msgs.msg import String
from mdr_speech_matching.msg import MatchedSentence
from ip_info.ip_info import IPInfo
from weather_api.weather_api import WeatherApi

class QuestionResponder(object):

    def __init__(self):
        rospy.init_node("question_responder")
        answer_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/answers.txt'))
        self.question_answer_pool = QuestionResponder.load_pool(answer_dir)
        self.questions = [i[0].strip() for i in self.question_answer_pool]
        self.answers = [i[1].strip() for i in self.question_answer_pool]
        self.sub = rospy.Subscriber("speech_matcher", MatchedSentence, self.respond)
        self.speech_request_topic = rospy.get_param('~speech_request_topic', '/say')
        self.pub = rospy.Publisher(self.speech_request_topic, String, latch=True, queue_size=1)

    @staticmethod
    def load_pool(filename):
        file = open(filename, "r")
        questions_and_answers = []
        for line in file:
            question_and_answer = line.split(":")
            questions_and_answers.append(question_and_answer)
        return questions_and_answers

    def respond(self, data):
        if data.type == MatchedSentence.TYPE_QUESTION:
            if data.matched_sentence == "how is the weather":
                coords = IPInfo.get_coordinates()
                weather = WeatherApi.get_weather(coords)
                if weather is not None:
                    # Successfully retrieved weather data
                    response = "It's {} at {} degrees".format(weather[0], weather[1])
                    rospy.loginfo("Found weather data: " + response)

                    respond = String()
                    respond.data = response
                    self.pub.publish(respond)
                    return
                else:
                    # We have no idea how the weather is, fall back to answers.txt
                    rospy.logerr("Could not retrieve weather data!")

            if data.matched_sentence == "where are you":
                location = IPInfo.get_location()
                if location is not None:
                    # We have a rough idea of where we are
                    response = "I'm near {} in {}".format(location[0], location[1])
                    rospy.loginfo("Answering with ip based location: " + response)

                    respond = String()
                    respond.data = response
                    self.pub.publish(respond)
                    return
                else:
                    # We have no idea where we are, fall back to answers.txt
                    rospy.logerr("Could not retrieve ip based location!")

            for i, question in enumerate(self.questions):
                if question == data.matched_sentence:

                    respond = String()
                    respond.data = self.answers[i]

                    rospy.loginfo("Your answer: '" + self.answers[i] + "'.")
                    self.pub.publish(respond)
                    return
            rospy.logerr("No answer found!")
        rospy.loginfo("This is not my type of sentence.")

def main():
    question_responder = QuestionResponder()
    rospy.spin()
