#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import String
import json


class Introduce(smach.State):

    def __init__(self, text):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.json1_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json"
        self.json2_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person2.json"
        self.text = text
        # self.pub = rospy.Publisher('/say_this', String, queue_size=10)

    def calculate_delay(self, text, length):
        """
        Calculate the delay for speech based on text length.
        """
        # Calculate delay based on text length
        num_words = len(text.split())
        if length == "long":
            delay = num_words * 0.6 
            delay = max(0.7, delay)
        else:
            delay = num_words * 0.2 
            delay = max(0.5, min(delay, 3.0))
        rospy.loginfo(f"Calculated delay: {delay} seconds")
        return delay

    def length_calculation(self,text):
        length = len(text.strip())
        if length > 15:
            return "long"
        else:
            return "short"



    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def execute(self, userdata):

        # rospy.loginfo("Robot says: %s", self.text)

        with open(self.json1_path) as f:
                    json1 = json.load(f)
                    person1 = json1["guest1"]

        with open(self.json2_path) as f:
            json2 = json.load(f)
            person2 = json2["guest1"]

        speech = f"{person1['name']} I would like to introduce you {person2['name']}, whose favorite drink is {person2['drink']}"
        self.text = speech
        msg = String()
        msg.data = self.text

        length = self.length_calculation(self.text)
        delay = self.calculate_delay(self.text, length)
        self.say_this(self.text)
        rospy.sleep(delay)  # wait while speaking
        rospy.loginfo("Robot says: %s", self.text)

        speech = f"{person2['name']} I would like to introduce you {person1['name']}, whose favorite drink is {person1['drink']}"
        self.text = speech
        msg = String()
        msg.data = self.text
        rospy.loginfo("Robot says: %s", self.text)

        length = self.length_calculation(self.text)
        delay = self.calculate_delay(self.text, length)
        self.say_this(self.text)
        rospy.sleep(delay)


        # rospy.sleep(1)  # allow publisher connection
        # self.pub.publish(msg)

        # rospy.sleep(2)  # wait while speaking

        return 'succeeded'


def main():

    rospy.init_node('say_this_state_machine')

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:

        smach.StateMachine.add(
            'SAY_HELLO',
            SayThis("Hello, I am the robot."),
            transitions={'succeeded': 'SAY_GOODBYE'}
        )

    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
