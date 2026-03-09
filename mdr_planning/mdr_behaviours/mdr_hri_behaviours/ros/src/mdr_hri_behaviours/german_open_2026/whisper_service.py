#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
# import whisper
from faster_whisper import WhisperModel
import subprocess
import os
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import cv2


class SpeechRecognitionService(ScenarioStateBase):
    _instances = {}

    @classmethod
    def get_instance(cls, instance_id="default", **kwargs):
        if instance_id not in cls._instances:
            cls._instances[instance_id] = cls(instance_id=instance_id, **kwargs)
        return cls._instances[instance_id]

    @classmethod
    def create_new_instance(cls, instance_id=None, **kwargs):
        if instance_id is None:
            instance_id = f"instance_{len(cls._instances)}"
        instance = cls(instance_id=instance_id, **kwargs)
        cls._instances[instance_id] = instance
        return instance

    def __init__(
        self,
        instance_id="default",
        transcript_topic="/speech/transcript",
        whisper_model_size="tiny.en",  # tiny.en, base.en, small.en, medium.en
        record_device="hw:1,0",        # default microphone
        record_duration=5, 
        device="cpu",            # seconds
    ):
        self.instance_id = instance_id
        self.record_device = record_device
        self.record_duration = int(record_duration)

        rospy.loginfo(f"[{self.instance_id}] Loading Whisper model ({whisper_model_size})...")
        self.whisper_model = WhisperModel(whisper_model_size, device=device, compute_type="float16")
        rospy.loginfo(f"[{self.instance_id}] Whisper model loaded.")

        # Publisher for transcripts
        self.transcript_pub = rospy.Publisher(transcript_topic, String, queue_size=10)

        # Advertise service
        self.service = rospy.Service(
            "speech_recognize", Trigger, self.handle_speech_recognition
        )

        rospy.loginfo(f"[{self.instance_id}] Speech Recognition Service ready.")

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    # def display_image(self,type_,delay):
    #     if type_ == "listen":
    #         path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/hri_utils/listen.jpg"
    #         window_name_ = "Listening"
        
    #     else:
    #         path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/hri_utils/speaking.jpg"
    #         window_name_ ="Speaking"
        
    #     image = cv2.imread(path)
    #             # Check if the image was loaded successfully
    #     if image is not None:
    #         window_name = window_name_
    #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    #         cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    #         cv2.imshow(window_name, image)
    #         cv2.waitKey(int(delay)) # Display for 3 seconds
    #         cv2.destroyWindow(window_name)

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

    def listen_and_transcribe(self):
        temp_filename = "/tmp/whisper_input.wav"

        record_cmd = [
            "arecord",
            "-D", "default",
            "-f", "S16_LE",
            "-r", "16000",
            "-c", "1",
            "-d", str(self.record_duration),
            temp_filename
        ]

        try:
            window_name, _ = self.display_image("listen")
            subprocess.run(record_cmd, check=True)
            if window_name:
                cv2.destroyWindow(window_name)

        except subprocess.CalledProcessError as e:
            rospy.logerr(f"[{self.instance_id}] arecord failed: {e}")
            if window_name:
                cv2.destroyWindow(window_name)
            return None

        try:
            # result = self.whisper_model.transcribe(
            #     temp_filename,
            #     language="en",
            #     temperature=0.0,
            #     no_speech_threshold=0.6
            # )
            # text = result["text"].strip()

            segments, info = self.whisper_model.transcribe(temp_filename, beam_size=5)
            text = " ".join([segment.text for segment in segments]).strip()

            
            if text:
                rospy.loginfo(f"[{self.instance_id}] Recognized: {text}")
                self.transcript_pub.publish(String(data=text))
                return text
            else:
                rospy.logwarn(f"[{self.instance_id}] Empty transcription.")
                self.say_this("I am having trouble understanding, can you please try again")
                rospy.sleep(0.3)
                
                return None
        except Exception as e:
            rospy.logerr(f"[{self.instance_id}] Whisper transcription error: {e}")
            return None
        finally:
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

    def handle_speech_recognition(self, req):
        """
        Service handler for speech recognition.
        """
        text = self.listen_and_transcribe()
        if text:
            return TriggerResponse(success=True, message=text)
        else:
            return TriggerResponse(success=False, message="No speech recognized")


if __name__ == "__main__":
    rospy.init_node("speech_to_text_service_node", anonymous=False)

    transcript_topic = rospy.get_param("~transcript_topic", "/speech/transcript")
    whisper_model_size = rospy.get_param("~whisper_model_size", "base.en")
    record_device = rospy.get_param("~record_device", "hw:1,0")
    record_duration = rospy.get_param("~record_duration", 6)

    stt_service = SpeechRecognitionService.get_instance(
        instance_id="default",
        transcript_topic=transcript_topic,
        whisper_model_size=whisper_model_size,
        record_device=record_device,
        record_duration=record_duration,
    )

    rospy.spin()