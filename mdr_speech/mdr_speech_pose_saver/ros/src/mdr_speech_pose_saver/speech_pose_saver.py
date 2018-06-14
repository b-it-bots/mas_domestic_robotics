from mdr_speech_pose_saver.pose_saver import PoseSaver
from std_msgs.msg import String
import rospy

class SpeechPoseSaver(PoseSaver):
    def __init__(self, file_name):
        rospy.init_node("speech_pose_saver")
        PoseSaver.__init__(self, file_name)
        rospy.Subscriber("speech_recognizer", String, self.recognizerCB)

    def recognizerCB (self, msg):
        #String must be filter Here
        self.setPoseName(msg.data)

        if not self.isExitRequested():
            self.savePose()

    def isExitRequested(self):
        if self.pose_name == "exit":
            return True

        return False
