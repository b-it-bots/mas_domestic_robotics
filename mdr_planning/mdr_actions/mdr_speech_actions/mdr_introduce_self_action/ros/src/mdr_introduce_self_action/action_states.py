#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import std_msgs

from mdr_introduce_self_action.msg import (IntroduceSelfFeedback,
                                           IntroduceSelfResult)

import datetime as dt

# TODO Can Jenny wave?
# TODO Greet to a recognized user "Hi Argen"
# TODO Add a response "Nice to meet you" if new, "How are you?" if known


class Greet(smach.State):
    # Say good morning, afternoon, evening.
    def __init__(self, say_topic='/sound/say'):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['introduce_self_goal'],
                             output_keys=['introduce_self_feedback',
                                          'introduce_self_result'])
        self.speak_pub = rospy.Publisher(say_topic,
                                         std_msgs.msg.String, latch=True,
                                         queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Greeting")

        feedback = IntroduceSelfFeedback()
        feedback.current_state = 'GREETING'
        feedback.message = '[introduce_self] greeting '
        userdata.introduce_self_feedback = feedback

        hour = dt.datetime.today().hour

        if 8 < hour < 12:
            self.speak_pub.publish("Guten Morgen!")
        elif hour < 18:
            self.speak_pub.publish("Guten Tag!")
        elif hour < 24:
            self.speak_pub.publish("Guten Abend!")
        else:
            self.speak_pub.publish("Hallo!")

        rospy.sleep(3)

        return 'succeeded'


class SayName(smach.State):
    def __init__(self, say_topic='/sound/say'):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['introduce_self_goal'],
                             output_keys=['introduce_self_feedback',
                                          'introduce_self_result'])
        self.speak_pub = rospy.Publisher(say_topic,
                                         std_msgs.msg.String, latch=True,
                                         queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Saying name")

        feedback = IntroduceSelfFeedback()
        feedback.current_state = 'SAY_NAME'
        feedback.message = '[introduce_self] saying name'
        userdata.introduce_self_feedback = feedback

        self.speak_pub.publish("My name is Jenny.")

        rospy.sleep(3)

        return 'succeeded'


# share ocupation, place of residence?
class ShareInformation(smach.State):
    def __init__(self, say_topic='/sound/say'):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['introduce_self_goal'],
                             output_keys=['introduce_self_feedback',
                                          'introduce_self_result'])
        self.speak_pub = rospy.Publisher(say_topic,
                                         std_msgs.msg.String, latch=True,
                                         queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Sharing", )

        feedback = IntroduceSelfFeedback()
        feedback.current_state = 'SHARE_INFORMATION'
        feedback.message = '[introduce_self] saying name'
        userdata.introduce_self_feedback = feedback

        if userdata.introduce_self_goal.profession:
            self.speak_pub.publish("I'm a full time robot at the Bonn-Rhein-Sieg University of Applied Sciences")
        if userdata.introduce_self_goal.residence:
            self.speak_pub.publish("I am from Sankt Augustin, Germany")
        if userdata.introduce_self_goal.date_of_birth:
            self.speak_pub.publish("I was manufactured in 2011", )

        rospy.sleep(3)

        return 'succeeded'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['introduce_self_goal'],
                             output_keys=['introduce_self_feedback',
                                          'introduce_self_result'])
        self.result = result

    def execute(self, userdata):
        result = IntroduceSelfResult()
        result.success = self.result
        userdata.introduce_self_result = result
        return 'succeeded'
