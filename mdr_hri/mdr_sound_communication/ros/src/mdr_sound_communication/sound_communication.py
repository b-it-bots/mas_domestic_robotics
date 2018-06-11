"""PyAudio Example: Play a wave file."""

import pyaudio
import wave
import sys
import yaml
import rospy
import rospkg
from dynamic_reconfigure.server import Server
from mdr_sound_communication.cfg import soundCommunicationConfig
from std_msgs.msg import String

class SoundCommunication:
    def __init__(self, config_file):
        package_name = "mdr_sound_communication"
        rospy.init_node(package_name + '_node', anonymous = False)

        self.package_path = rospkg.RosPack().get_path(package_name) + '/'

        list_config_files = rospy.get_param("~config_files", ["topic_config"])
        print list_config_files
        list_config_files = list_config_files.split(" ")
        self.sound_dictionary = dict()

        for config_file in list_config_files:
            print config_file
            self.sound_dictionary.update(yaml.load(open(self.package_path + "config/" + config_file+".yaml")))
        self.CHUNK = 1024 #TODO

        self.is_enable = True
        self.audio_manager = pyaudio.PyAudio()

        for sound in self.sound_dictionary:
            try:
                sound_file = self.sound_dictionary[sound]['file']
                sound_topic = self.sound_dictionary[sound]['topic']
                rospy.Subscriber(sound_topic, rospy.AnyMsg, self.mainCB, sound_file)
                rospy.loginfo("Sound %s linked to topic %s", sound_file, sound_topic)
            except Exception:
                rospy.logdebug("Ignoring Topic Linking of sound %s", self.sound_dictionary[sound])

        rospy.Subscriber('sound_monitor', String, self.soundCB)
        self.dyn_reconfigure_srv = Server(soundCommunicationConfig, self.dynamic_reconfigureCB)

        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.is_enable = config["allow_sound"]
        return config

    def soundCB(self,msg):
        sound = msg.data

        try:
            sound_file = self.sound_dictionary[sound]['file']
            sound_file_path = self.package_path + 'willow-sound/'+ sound_file
            if self.is_enable:
                self.playSound(sound_file_path)

        except Exception:
            sound_file = self.sound_dictionary[sound]
            sound_file_path = self.package_path + 'willow-sound/'+ sound_file
            if self.is_enable:
                self.playSound(sound_file_path)

        except:
            rospy.logerr('Required Sound does not exists')

    def mainCB(self, msg, sound_file):
        sound_file_path = self.package_path + 'willow-sound/'+ sound_file
        if self.is_enable:
            self.playSound(sound_file_path)

    def playSound(self,sound_file):
        try:
            wf = wave.open(sound_file, 'rb')
            data = wf.readframes(self.CHUNK)
            stream = self.audio_manager.open(format=self.audio_manager.get_format_from_width(wf.getsampwidth()),
                          channels=wf.getnchannels(),
                          rate=wf.getframerate(),
                          output=True)

            while len(data) > 0:
                stream.write(data)
                data = wf.readframes(self.CHUNK)

            # stop stream (4)
            stream.stop_stream()
            stream.close()

        except Exception:
            rospy.logerr('Required Sound does not exists')
