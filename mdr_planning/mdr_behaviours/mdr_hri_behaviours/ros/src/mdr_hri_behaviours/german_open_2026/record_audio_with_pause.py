#!/usr/bin/env python3

from std_msgs.msg import String, Bool
import rospy
from std_msgs.msg import String
import whisper
import subprocess
import os


class SpeechRecognitionService:
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
        whisper_model_size="base.en",   # tiny.en, base.en, small.en, medium.en
        record_device="hw:1,0",         # PrimeSense mic on HSR
        record_duration=5,              # seconds
    ):
        self.instance_id = instance_id
        self.record_device = record_device
        self.record_duration = int(record_duration)
        self.record_enabled = False

        # Load Whisper model (offline)
        rospy.loginfo(f"[{self.instance_id}] Loading Whisper model ({whisper_model_size})...")
        self.whisper_model = whisper.load_model(whisper_model_size)
        rospy.loginfo(f"[{self.instance_id}] Whisper model loaded.")

        rospy.loginfo(f"[{self.instance_id}] Speech Recognition Service initialized.")

        # Publishers
        # self.say_pub = rospy.Publisher("/say", String, queue_size=10)
        self.transcript_pub = rospy.Publisher(transcript_topic, String, queue_size=10)
        
        # Subcribers
        self.record_sub = rospy.Subscriber("/condition_record", Bool, self.condition_callback)

    def say_this(self, message):
        self.say_pub.publish(String(data=message))
        rospy.loginfo(f"Saying: {message}")

    def condition_callback(self, msg):
        self.record_enabled = msg.data
        # rospy.loginfo(f"[{self.instance_id}] Recording enabled: {self.record_enabled}")

    # def listen_and_transcribe(self):
    #     rospy.loginfo(f"[{self.instance_id}] Recording {self.record_duration}s...")

    #     # temp_file = "/tmp/whisper_input.wav"

    #     # record_cmd = [
    #     #     "arecord",
    #     #     "-D", self.record_device,
    #     #     "-f", "S16_LE",
    #     #     "-r", "16000",
    #     #     "-c", "2",
    #     #     "-d", str(self.record_duration),
    #     #     "-q",                  # quiet mode (important)
    #     #     temp_file
    #     # ]

    #     # Use plughw and auto-detect channels
    #     temp_filename = "/tmp/whisper_input.wav"

    #     # Auto detect number of channels for hw:1,0
    #     channels = 1  # fallback mono
    #     try:
    #         out = subprocess.check_output(["arecord", "-D", "hw:1,0", "--dump-hw-params"], universal_newlines=True)
    #         if "CHANNELS" in out:
    #             for line in out.splitlines():
    #                 if line.strip().startswith("CHANNELS:"):
    #                     channels = int(line.split(":")[1].strip())
    #     except Exception:
    #         channels = 1

    #     record_cmd = [
    #         "arecord",
    #         "-D", "plughw:1,0",       # plughw auto-converts
    #         "-f", "S16_LE",
    #         "-r", "16000",
    #         "-c", str(channels),
    #         "-d", "3",                 # shorten duration to avoid xruns
    #         temp_filename
    #     ]

    #     try:
    #         subprocess.run(record_cmd, check=True)

    #         result = self.whisper_model.transcribe(
    #             temp_filename,
    #             language="en",
    #             temperature=0.0
    #         )

    #         text = result["text"].strip()

    #         if text:
    #             rospy.loginfo(f"[{self.instance_id}] Recognized: {text}")
    #             self.transcript_pub.publish(String(data=text))
    #             return text
    #         else:
    #             rospy.logwarn(f"[{self.instance_id}] Empty transcription.")
    #             return None

    #     except subprocess.CalledProcessError:
    #         rospy.logerr(f"[{self.instance_id}] arecord failed.")
    #         return None
    #     except Exception as e:
    #         rospy.logerr(f"[{self.instance_id}] Whisper error: {e}")
    #         return None
    #     finally:
    #         if os.path.exists(temp_filename):
    #             os.remove(temp_filename)


    def listen_and_transcribe(self):
        rospy.loginfo(f"[{self.instance_id}] Recording 5s...")

        temp_filename = "/tmp/whisper_input.wav"

        # Recommended: mono, let ALSA convert if needed
        record_cmd = [
            "arecord",
            "-D", "default",     # correct device
            "-f", "S16_LE",      # format
            "-r", "16000",       # sample rate
            "-c", "1",           # mono
            "-d", "5",           # duration in seconds
            temp_filename
        ]

        try:
            subprocess.run(record_cmd, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"[{self.instance_id}] arecord failed: {e}")
            return None

        # Transcribe with Whisper
        try:
            print("In the trasnscribing loop")
            result = self.whisper_model.transcribe(
                temp_filename,
                language="en",
                temperature=0.0,
                no_speech_threshold=0.6
            )
            text = result["text"].strip()
            if text:
                rospy.loginfo(f"[{self.instance_id}] Recognized: {text}")
                self.transcript_pub.publish(String(data=text))
                return text
            else:
                rospy.logwarn(f"[{self.instance_id}] Empty transcription.")
                return None
        except Exception as e:
            rospy.logerr(f"[{self.instance_id}] Whisper transcription error: {e}")
            return None
        finally:
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

    def run(self, rate_hz=10):
        rate = rospy.Rate(rate_hz)
        rospy.loginfo(f"[{self.instance_id}] Running Whisper STT loop...")
        while not rospy.is_shutdown():
            # rospy.loginfo(f"[Status] {self.record_enabled}")
            if self.record_enabled:
                self.listen_and_transcribe()
            else:
                rospy.logdebug(f"[{self.instance_id}] Recording paused")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("speech_to_text_node", anonymous=False)

    transcript_topic = rospy.get_param("~transcript_topic", "/speech/transcript")
    whisper_model_size = rospy.get_param("~whisper_model_size", "base.en")
    record_device = rospy.get_param("~record_device", "hw:1,0")
    record_duration = rospy.get_param("~record_duration", 5)
    rate_hz = rospy.get_param("~rate", 1)

    stt = SpeechRecognitionService.get_instance(
        instance_id="default",
        transcript_topic=transcript_topic,
        whisper_model_size=whisper_model_size,
        record_device=record_device,
        record_duration=record_duration,
    )

    stt.run(rate_hz=rate_hz)