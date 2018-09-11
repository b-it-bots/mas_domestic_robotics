import rospy

from std_msgs.msg import String

class SoundVocaliserBase(object):
    def __init__(self):
        self.speech_request_topic = rospy.get_param('~speech_request_topic', '/say')
        self.sound_request_topic = rospy.get_param('~sound_request_topic', '/make_sound')
        self.speech_topic = rospy.get_param('~speech_topic', '/sound/say')
        self.sound_topic = rospy.get_param('~sound_topic', '/sound/make')

        self.speech_request_sub = rospy.Subscriber(self.speech_request_topic,
                                                   String,
                                                   self.say)
        self.sound_request_sub = rospy.Subscriber(self.sound_request_topic,
                                                  String,
                                                  self.make_sound)

    def say(self, msg):
        rospy.loginfo('[SAY] Ignoring request')

    def make_sound(self, msg):
        rospy.loginfo('[MAKE_SOUND] Ignoring request')
