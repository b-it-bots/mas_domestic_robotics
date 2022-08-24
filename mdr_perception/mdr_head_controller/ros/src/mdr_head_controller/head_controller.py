import rospy
from mdr_head_controller.head_controller_base import HeadControllerBase


class HeadController(HeadControllerBase):
    def __init__(self):
        self.error_message = "this is only a dummy class, please use robot-specific package to control the head"
        rospy.logerr(self.error_message)

    def look_up(self):
        rospy.logerr(self.error_message)
        rospy.loginfo("(DUMMY) look up")

    def look_down(self):
        rospy.logerr(self.error_message)
        rospy.loginfo("(DUMMY) look down")

    def turn_left(self):
        rospy.logerr(self.error_message)
        rospy.loginfo("(DUMMY) turn left")

    def turn_right(self):
        rospy.logerr(self.error_message)
        rospy.loginfo("(DUMMY) turn right")
