#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import ollama
import queue
import json
import re
from mdr_hri_behaviours.srv import Prompt
from std_srvs.srv import Trigger, TriggerRequest
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import cv2

class GuestInformation (ScenarioStateBase):
    def __init__(self, in_topic="/speech/transcript", max_queue=50,):
        rospy.loginfo("Initializing GuestInformationROS ....")

        # ROS I/O
        # self.in_sub = rospy.Subscriber(in_topic, String, self._on_transcript, queue_size=10)
        self.mic_control_pub = rospy.Publisher("/condition_record", Bool, queue_size=10)
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)

        #ROS service
        rospy.wait_for_service("voicebot/prompt")

        self.proxy = rospy.ServiceProxy("voicebot/prompt", Prompt)

        # Whisper STT service
        rospy.wait_for_service("speech_recognize")

        self.stt_service = rospy.ServiceProxy("speech_recognize", Trigger)

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

    def display_image(self,type_,delay):
        if type_ == "listen":
            path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/hri_utils/listen.jpg"
            window_name_ = "Listening"
        
        else:
            path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/hri_utils/speaking.jpg"
            window_name_ ="Speaking"
        
        image = cv2.imread(path)
                # Check if the image was loaded successfully
        if image is not None:
            window_name = window_name_
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(window_name, image)
            cv2.waitKey(int(delay)) # Display for 3 seconds
            cv2.destroyWindow(window_name)

    

    def execute(self):
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
                self.display_image("listen",6900)
                

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

                with open(
                    "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json",
                    "w",
                ) as f:
                    json.dump(json_data, f, indent=4)

                rospy.loginfo("Guest information saved.")
                # say the reply from the llm after saving
                length = self.length_calculation(reply)
                sleep_time = self.calculate_delay(reply,length)
                self.say_this(reply)
                self.display_image("speaking",sleep_time)
                rospy.sleep(sleep_time)

                break

            else:
                rospy.logwarn("Name or drink missing. Asking again...")
                length = self.length_calculation(reply)
                sleep_time = self.calculate_delay(reply,length)
                self.say_this(reply)
                self.display_image("speaking",sleep_time)
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


# def call_speech_service(self):
    #     # speech_service = rospy.ServiceProxy('/speech_recognize', Trigger)
    #     req = TriggerRequest()
    #     response = None

    #     while not rospy.is_shutdown():  # keep looping until ROS shuts down
    #         try:
    #             rospy.loginfo("Calling speech recognition service...")
    #             response = self.stt_service(req)

    #             if response.success:
    #                 rospy.loginfo(f"Transcription received: {response.message}")
    #                 break  # exit the loop if we got a successful response
    #             else:
    #                 rospy.logwarn("No transcription. Retrying...")
    #                 rospy.sleep(1.0)  # wait 1 second before retrying

    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call failed: {e}")
    #             rospy.sleep(1.0)  # wait a bit before retrying

    #     return response.message if response else ""


    # def execute(self):

    #     try:
    #         #say_this("Hello, welcome My name is Lucy")
    #         #rospy.sleep(2)
    #         rospy.loginfo("Calling speech recognition service...")
    #         speech_service = rospy.ServiceProxy('/speech_recognize', Trigger)
    #         req = TriggerRequest()
    #         response = speech_service(req)
    #         if response.success:
    #             rospy.loginfo(f"Transcription: {response.message}")
    #         else:
    #             #say_this("I am sorry I cant hearyou can you repaet again?")
    #             #rospy.sleep(2)
    #             rospy.logwarn("I am sorry I cant hearyou can you repaet again?")
    #             rospy.logwarn("Failed to recognize speech: " + response.message)

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")
        
        
    #     text = (response.message or "").strip()
    #     if not text:
    #         return
    #     try:
    #         self.mic_control_pub.publish(Bool(data=False))
    #         rospy.loginfo(f"Sending prompt: {text}")

    #         result = self.proxy(text)

    #         guests = result.guests_json


    #        # Guest 1
    #         if guests["guest1"]["name"] and guests["guest1"]["drink"]:

    #             json_data = {
    #                 "guest1": guests["guest1"]
    #             }
    #             with open("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json", "w") as f:
    #                 json.dump(json_data, f, indent=4)

    #         if guests["guest2"]["name"] and guests["guest2"]["drink"]:

    #             json_data = {
    #                 "guest2": guests["guest2"]
    #             }
    #             with open("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person2.json", "w") as f:
    #                 json.dump(json_data, f, indent=4)

    #         self.mic_control_pub.publish(Bool(data=True))

    #     except rospy.ServiceException as e:

    #         rospy.logwarn(f"Service call failed: {e}")

    # def execute(self):
    #     """Continuously call the speech service until a valid transcription is received,
    #     then send the prompt and save guest information to JSON files.
    #     """
    #     # ---------------- Keep trying speech recognition ----------------
    #     speech_service = rospy.ServiceProxy('/speech_recognize', Trigger)
    #     req = TriggerRequest()
    #     response = None

    #     while not rospy.is_shutdown():
    #         try:
    #             rospy.loginfo("Calling speech recognition service...")
    #             response = speech_service(req)

    #             if response.success and response.message.strip():
    #                 rospy.loginfo(f"Transcription received: {response.message}")
    #                 break  # exit the loop on successful transcription
    #             else:
    #                 rospy.logwarn("No transcription or empty message. Retrying...")
    #                 rospy.sleep(1.0)

    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call failed: {e}")
    #             rospy.sleep(1.0)

    #     text = response.message.strip()
    #     if not text:
    #         rospy.logwarn("No valid transcription obtained. Exiting execute.")
    #         return

    #     # ---------------- Disable mic while processing ----------------
    #     self.mic_control_pub.publish(Bool(data=False))

    #     # ---------------- Send prompt to voicebot ----------------
    #     try:
    #         rospy.loginfo(f"Sending prompt to voicebot: {text}")
    #         result = self.proxy(text)
    #         guests = result.guests_json

    #         # ---------------- Save guest1 info if available ----------------
    #         if guests.get("guest1") and guests["guest1"].get("name") and guests["guest1"].get("drink"):
    #             json_data = {"guest1": guests["guest1"]}
    #             with open("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json", "w") as f:
    #                 json.dump(json_data, f, indent=4)
    #             rospy.loginfo("Saved guest1 info to JSON.")

    #         # ---------------- Save guest2 info if available ----------------
    #         if guests.get("guest2") and guests["guest2"].get("name") and guests["guest2"].get("drink"):
    #             json_data = {"guest2": guests["guest2"]}
    #             with open("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person2.json", "w") as f:
    #                 json.dump(json_data, f, indent=4)
    #             rospy.loginfo("Saved guest2 info to JSON.")

    #     except rospy.ServiceException as e:
    #         rospy.logwarn(f"Voicebot service call failed: {e}")

    #     # ---------------- Re-enable mic ----------------
    #     self.mic_control_pub.publish(Bool(data=True))