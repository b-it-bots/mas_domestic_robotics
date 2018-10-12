#!/usr/bin/env python
from __future__ import print_function
import rospy
import os
from tmc_msgs.msg import Voice
from mdr_speech_matching.msg import MatchedSentence

class QuestionResponder():

    def __init__(self):
        answer_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/answers.txt'))
        self.question_answer_pool = QuestionResponder.load_pool(answer_dir)
        self.questions = [i[0].strip() for i in self.question_answer_pool]
        self.answers = [i[1].strip() for i in self.question_answer_pool]
        self.sub = rospy.Subscriber("speech_matcher", MatchedSentence, self.respond)
        self.pub = rospy.Publisher("/talk_request", Voice, latch=True, queue_size=1)

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
            for i, question in enumerate(self.questions):
                if question == data.matched_sentence:

                    respond = Voice()
                    respond.queueing = False
                    respond.language = 1
                    respond.sentence = self.answers[i]

                    rospy.loginfo("Your answer: '" + self.answers[i] + "'.")
                    self.pub.publish(respond)
                    return
            rospy.logerr("No answer found!")
        rospy.loginfo("This is not my type of sentence.")

def main():
    rospy.init_node("question_responder")
    question_responder = QuestionResponder()
    rospy.spin()
