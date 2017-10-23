#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2017.10.10

@author: Patrick Nagel
"""

import sys
import rospy
import smach
from smach import cb_interface
import smach_ros

from mdr_speech_actions.msg import ListenAction, ListenGoal, AskAction, AnswerAction, AnswerGoal, AskGoal
from smach_ros import SimpleActionState

def print_usage_info():
    rospy.loginfo("usage: speech_demo <duration>")
    rospy.loginfo("       where the <duration> is measured in seconds and defines "\
    "how long the robot is in listen-mode")
    rospy.sleep(5)
    return raw_input("Enter a duration and confirm with the return key: ")

def main():
    rospy.init_node('speech_demo')

    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])

    with sm:
        listen_goal = ListenGoal()
        try:
            user_input_duration = sys.argv[1]
            user_input_duration = int(user_input_duration)
            listen_goal.listen_duration = user_input_duration
        except IndexError:
            rospy.logerr("Received unknown value for <duration> argument")
            user_input_duration = print_usage_info()
            user_input_duration = int(user_input_duration)
            listen_goal.listen_duration = user_input_duration
        except ValueError:
            rospy.logerr("Received unknown value for <duration> argument")
            user_input_duration = print_usage_info()
            user_input_duration = int(user_input_duration)
            listen_goal.listen_duration = user_input_duration
        except:
            rospy.logerr("You have failed!")

        @cb_interface(outcomes=['question', 'questionable_statement'])

        def check_result(userdata, status, result):
            rospy.loginfo('Message: %s', result.message)
            rospy.loginfo('Message type: %s', result.message_type)
            if result.message_type == "question":
                return 'question'
            if result.message_type == "questionable_statement":
                return 'questionable_statement'

        def give_answer_goal(userdata, goal):
            answer_goal = AnswerGoal()
            answer_goal.question = goal.question
            return answer_goal

        def give_ask_goal(userdata, goal):
            ask_goal = AskGoal()
            ask_goal.triggering_statement = goal.triggering_statement

        smach.StateMachine.add('LISTEN', SimpleActionState('listen_server', ListenAction,\
        goal=listen_goal, result_cb=check_result, result_slots=['message', 'message_type',\
        'success']), transitions={'succeeded':'SUCCESS', 'preempted':'FAIL', 'aborted':'FAIL',\
        'question':'ANSWER', 'questionable_statement':'ASK'}, \
        remapping={'message':'user_message', 'message_type':'user_messeage_type',\
        'success':'user_success'})

        smach.StateMachine.add('ASK', SimpleActionState('ask_server', AskAction,\
        goal_cb=give_ask_goal, goal_slots=['triggering_statement']), \
        transitions={'succeeded':'SUCCESS', 'preempted':'FAIL', 'aborted':'FAIL'},\
        remapping={'triggering_statement':'user_message'})

        smach.StateMachine.add('ANSWER', SimpleActionState('answer_server', AnswerAction,\
        goal_cb=give_answer_goal, goal_slots=['question']),\
        transitions={'succeeded':'SUCCESS', 'preempted':'FAIL', 'aborted':'FAIL'},\
        remapping={'question':'user_message'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
