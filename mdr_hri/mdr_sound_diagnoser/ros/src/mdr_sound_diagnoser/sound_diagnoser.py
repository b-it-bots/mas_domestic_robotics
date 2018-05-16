"""PyAudio Example: Play a wave file."""

import pyaudio
import wave
import sys
import yaml
import rospy
import rospkg
from dynamic_reconfigure.server import Server
from mdr_sound_diagnoser.cfg import soundDiagnoserConfig


class SoundDiagnoser:
    def __init__(self, config_file):
        package_name = "mdr_sound_diagnoser"
        rospy.init_node(package_name + '_node', anonymous = False)

        self.package_path = rospkg.RosPack().get_path(package_name) + '/'
        sound_dictionary = yaml.load(open(self.package_path + "config/" + config_file+".yaml"))
        self.CHUNK = 1024 #TODO

        self.is_enable = True
        self.audio_manager = pyaudio.PyAudio()

        for sound in sound_dictionary:
            sound_file = sound_dictionary[sound]['folder']+'/'+ sound_dictionary[sound]['file_name']
            rospy.Subscriber(sound_dictionary[sound]['topic'], rospy.AnyMsg, self.mainCB, sound_file)

        self.dyn_reconfigure_srv = Server(soundDiagnoserConfig, self.dynamic_reconfigureCB)

        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.is_enable = config["allow_sound"]
        return config

    def mainCB(self, msg, sound_file):
        sound_file_path = self.package_path + 'willow-sound/'+ sound_file
        if self.is_enable:
            self.playSound(sound_file_path)

    def playSound(self,sound_file):
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
