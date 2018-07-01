from mdr_speech_pose_saver.pose_saver import PoseSaver
from std_msgs.msg import String
import rospy

class SpeechPoseSaver(PoseSaver):
    def __init__(self):
        rospy.init_node("speech_pose_saver_robot_command")
        map_frame = rospy.get_param('~map_frame', 'map')
        file_name = rospy.get_param('file_name', 'navigation_goals.yaml')
        robot_frame = rospy.get_param('~robot_frame', 'base_link')
        speech_input_topic = rospy.get_param('~speech_input_topic', 'speech_recognizing')
        PoseSaver.__init__(self, file_name, map_frame, robot_frame)
        self.is_pose_requested = False
        self.exit_requested = False
        rospy.Subscriber(speech_input_topic, String, self.recognizerCB)
        #rospy.spinOnce()

    def recognizerCB (self, msg):

        if msg.data == "exit":
            rospy.logwarn("Exit Requested")
            self.exit_requested = True

        if "confirm" in msg.data and self.is_pose_requested:
            if not self.is_exit_requested():
                self.save_pose()
                return
        else:
            if self.is_pose_requested:
                rospy.logwarn("Discarting Pose")
                self.is_pose_requested = False

        #String must be filter
        if "save pose" in msg.data and not self.is_pose_requested:
            self.set_pose_name(msg.data)
            rospy.loginfo("set pose to save as %s", self.get_pose_name())
            self.is_pose_requested = True

    def is_exit_requested(self):
        return self.exit_requested
