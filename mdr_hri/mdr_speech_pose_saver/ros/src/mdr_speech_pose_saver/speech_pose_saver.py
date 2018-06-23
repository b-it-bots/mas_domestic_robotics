from mdr_speech_pose_saver.pose_saver import PoseSaver
from std_msgs.msg import String
import rospy

class SpeechPoseSaver(PoseSaver):
    def __init__(self, file_name):
        rospy.init_node("speech_pose_saver")
        map_frame = rospy.get_param('~map_frame', 'map')
        robot_frame = rospy.get_param('~robot_frame', 'base_link')
        speech_input_topic = rospy.get_param('~speech_input_topic', 'speech_recognizing')
        PoseSaver.__init__(self, file_name, map_frame, robot_frame)
        rospy.Subscriber(speech_input_topic, String, self.recognizerCB)

    def recognizerCB (self, msg):
        #String must be filter Here
        self.set_pose_name(msg.data)

        if not self.is_exit_requested():
            self.save_pose()

    def is_exit_requested(self):
        if self.pose_name == "exit":
            return True

        return False
