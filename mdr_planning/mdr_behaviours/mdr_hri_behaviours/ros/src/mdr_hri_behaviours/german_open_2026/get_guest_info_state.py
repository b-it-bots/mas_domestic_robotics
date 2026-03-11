#!/usr/bin/env python3

#~/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/

import rospy
from std_msgs.msg import String, Bool
import ollama
import queue
import json
import re
# from mdr_hri_behaviours.srv import Prompt, ComparePerson
from llm_server.srv import Prompt, ComparePerson
from std_srvs.srv import Trigger, TriggerRequest
# from mas_execution_manager.scenario_state_base import ScenarioStateBase
import os
import cv2
import smach

class GuestInformation (smach.State): #ScenarioStateBase
    def __init__(self, in_topic="/speech/transcript", max_queue=50,):
        smach.State.__init__(
            self,
            outcomes=['guest1_info_saved', 'guest2_info_saved'],
        )
        rospy.loginfo("Initializing GuestInformationROS ....")

        # ROS I/O
        # self.in_sub = rospy.Subscriber(in_topic, String, self._on_transcript, queue_size=10)
        self.mic_control_pub = rospy.Publisher("/condition_record", Bool, queue_size=10)
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)

        #ROS service
        rospy.wait_for_service("voicebot/prompt")

        self.proxy = rospy.ServiceProxy("voicebot/prompt", Prompt)

        rospy.loginfo("GuestInformationROS llm.")

        # Whisper STT service
        rospy.wait_for_service("speech_recognize")

        self.stt_service = rospy.ServiceProxy("speech_recognize", Trigger)



        rospy.loginfo("GuestInformationROS speech.")
        # rospy.wait_for_service("/compare_person")

        # self.person_recognise = rospy.ServiceProxy("/compare_person", ComparePerson)


        # Mic state: True means "listening", False means "processing"
        self.mic_state = True
        rospy.loginfo("GuestInformationROS initialized.")

        # ---------------- Mic state control ----------------
    def set_mic(self, state: bool):
        """Update mic state variable; spin loop will continuously publish."""
        print(f"Setting mic state to: {state}")
        self.mic_state = state

    def publish_mic_state(self):
        """Publish current mic state."""
        self.mic_control_pub.publish(Bool(data=self.mic_state))

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


    def display_image(self, type_):
        if type_ == "listen":
            path = "/home/lucy/.../listen.jpg"
            window_name = "Listening"
        else:
            path = "/home/lucy/.../speaking.jpg"
            window_name = "Speaking"

        image = cv2.imread(path)
        if image is None:
            rospy.logwarn("Image not found: " + path)
            return None, None

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(window_name, image)
        cv2.waitKey(1)  # required to display
        return window_name, image


    

    def execute(self, userdata=None):
        """
        Keep calling Whisper speech recognition until a guest name and drink
        are successfully extracted from the voicebot.
        """

        speech_service = rospy.ServiceProxy('/speech_recognize', Trigger)
        req = TriggerRequest()

        while not rospy.is_shutdown():

            # ---------------- Call Whisper STT ----------------
            try:
                rospy.loginfo("Calling speech recognition service...")
                response = speech_service(req)
                self.display_image("listen")
                

                if not response.success or not response.message.strip():
                    rospy.logwarn("No transcription received. Retrying...")
                    rospy.sleep(1.0)
                    continue

                text = response.message.strip()
                rospy.loginfo(f"Transcription: {text}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Speech service failed: {e}")
                rospy.sleep(1.0)
                continue


            # ---------------- Disable mic while processing ----------------
            self.mic_control_pub.publish(Bool(data=False))

            # ---------------- Send text to voicebot ----------------
            try:
                rospy.loginfo(f"Sending prompt to voicebot: {text}")
                result = self.proxy(text)
                reply = result.response
                # guests = result.guests_json
                guests = json.loads(result.guests_json)
                print(type(guests))

            except rospy.ServiceException as e:
                rospy.logwarn(f"Voicebot service failed: {e}")
                self.mic_control_pub.publish(Bool(data=True))
                rospy.sleep(1.0)
                continue


            # ---------------- Check if we got valid info ----------------
            guest1 = guests.get("guest1", {})

            name = guest1.get("name")
            drink = guest1.get("drink")

            if name and drink:

                rospy.loginfo(f"Guest extracted: {name} wants {drink}")

                json_data = {"guest1": guest1}

                # json1_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json"
                json1_path = "/home/sun/catkin_ws/temp/person1.json"

                if os.path.isfile(json1_path):
                    # file_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person2.json"
                    file_path = "/home/sun/catkin_ws/temp/person2.json"
                else:
                    file_path = json1_path

                with open(
                    file_path,
                    "w",
                ) as f:
                    json.dump(json_data, f, indent=4)

                rospy.loginfo("Guest information saved.")
                # say the reply from the llm after saving
                length = self.length_calculation(reply)
                sleep_time = self.calculate_delay(reply,length)
                self.say_this(reply)
                self.display_image("speaking")
                rospy.sleep(sleep_time)

                if file_path == json1_path:
		    reply = "Please follow me to the setting area and have a seat"
		    length = self.length_calculation(reply)
                    sleep_time = self.calculate_delay(reply,length)
                    self.say_this(reply)
                    self.display_image("speaking")
		    return 'guest1_info_saved'
                else:                  
		    reply = "Please follow me to the setting area and have a seat"  
		    length = self.length_calculation(reply)
                    sleep_time = self.calculate_delay(reply,length)
                    self.say_this(reply)
                    self.display_image("speaking")
                    return 'guest2_info_saved'

            else:
                rospy.logwarn("Name or drink missing. Asking again...")
                length = self.length_calculation(reply)
                sleep_time = self.calculate_delay(reply,length)
                self.say_this(reply)
                self.display_image("speaking")
                rospy.sleep(sleep_time)

                self.mic_control_pub.publish(Bool(data=True))
                rospy.sleep(1.0)


        # ---------------- Re-enable mic ----------------
        self.mic_control_pub.publish(Bool(data=True))


def main():

    rospy.init_node("guestinformation_ros")

    gi = GuestInformation()
    gi.execute()
    rospy.spin()


if __name__ == "__main__":
    main()
