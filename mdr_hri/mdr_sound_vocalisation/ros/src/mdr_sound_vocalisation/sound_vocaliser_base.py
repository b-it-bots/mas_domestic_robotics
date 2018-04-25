import rospy

class SoundVocaliserBase(object):
    def __init__(self):
        self.speech_request_topic = rospy.get_param('~speech_topic', '/say')
        self.sound_request_topic = rospy.get_param('~sound_topic', '/make_sound')
        self.speech_topic = rospy.get_param('~speech_topic', '/sound/say')
        self.sound_topic = rospy.get_param('~sound_topic', '/sound/make')

    def say(self, mgs):
        rospy.loginfo('[SAY] Ignoring request')

    def make_sound(self, msg):
        rospy.loginfo('[MAKE_SOUND] Ignoring request')