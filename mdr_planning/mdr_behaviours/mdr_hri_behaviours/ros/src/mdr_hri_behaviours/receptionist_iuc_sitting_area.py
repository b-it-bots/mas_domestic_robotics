"""
Receptionist Challenge -2024
Authors: Ayusee Swain
ayusee.1998@gmail.com
"""

import rospy
import speech_recognition as sr
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import String  # ROS standard message

class GuestIntroductionTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'guest_sitting_area_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['persons_record'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        self.person_data=None

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def execute(self, userdata):
        rospy.loginfo("Initiating guest introduction...")
        self.person_data=userdata.persons_record[-1]
        guest_name=self.person_data["guest_name"]
        favorite_drink=self.person_data["favorite_drink"]
        # guest_name = userdata.guest_name if 'guest_name' in userdata else "the guest"
        # favorite_drink = userdata.favorite_drink if 'favorite_drink' in userdata else "their favorite drink"

        introduction_phrase = f"{guest_name}, please be seated on the couch. It was a pleasure assisting you."

        #introduction_phrase = f"Hi Everyone, please meet {guest_name}. Their favorite drink is {favorite_drink}. It was a pleasure assisting you today. Have a wonderful time. Goodbye!"
        
        self.say_this(introduction_phrase)
        rospy.sleep(2)  # Give some time for people in the room to acknowledge the introduction

        rospy.loginfo("Guest introduced successfully.")
        return 'succeeded'
