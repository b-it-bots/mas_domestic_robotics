#!/usr/bin/env python
__author__ = 'Iryna'

import rospy
import std_msgs.msg

pub_answer = rospy.Publisher("question_matcher/answer", std_msgs.msg.String, queue_size=1)

'''NOTE: Questions and answers are being loaded from a txt file.'''
'''RoboCup2016: There are 50 questions given. Give an answer to 5 out of 50 in 5 min.'''


def search_items(recognized_phrase, array):
    x = []
    i = ''
    for item in array:
        if recognized_phrase.find(item) != -1:
            i = item
            x.append(item)
    return x


def sentence_matching(recognized_phrase, questions):

    candidates = []
    numbers = []
    winner = ""

    recognized_phrase = str(recognized_phrase.lower())
    words = recognized_phrase.split()

    for question in questions:

        question = str(question.lower())
        number_question = len(question.split())

        keys = search_items(question, words)
        number_keys = len(keys)

        if number_keys == number_question: # total match
            #print number_keys
            winner = question
            return winner

        elif number_keys != number_question:
            numbers.append(number_keys)
            candidates.append(question)
            winner = ""

    #print numbers

    if winner == "":
        if max(numbers) > 0:
            winner = candidates[numbers.index(max(numbers))]
        else:
            winner = ""

    return winner


def question_answer_matching(recognized_phrase):

    txtfile = rospy.get_param("~questions_file")
    
    questions_answers = {}
    questions = []

    with open(txtfile) as f:
        for line in f:
            line = line.lower()
            line = line.strip()
            line = line.rstrip(',')
            try:
                (key,val) = line.split(":")
                questions_answers[str(key)] = str(val)
                if str(key) != '':
                    questions.append(str(key))
            except ValueError:
                print('Ignoring: malformed line: "{}"'.format(line))

    winner = sentence_matching(recognized_phrase, questions)

    if not questions_answers[winner]:
        #print("Unknown question!")
        answer = "I wasn't able to recognize your question!" #random answer

    elif questions_answers[winner]:
        # print(questions_answers[winner])
        answer = questions_answers[winner]

    else:
        print("Something bad happened...")
        answer = "I wasn't able to recognize your question!" #random answer

    return answer

def recognized_text_callback(recognized_phrase):
    answer = question_answer_matching(recognized_phrase.data)
    pub_answer.publish(answer)
    

def main():
    rospy.init_node('question_matcher', anonymous=True)
    sub = rospy.Subscriber('~recognized_speech', std_msgs.msg.String, recognized_text_callback)
    rospy.spin()
    
