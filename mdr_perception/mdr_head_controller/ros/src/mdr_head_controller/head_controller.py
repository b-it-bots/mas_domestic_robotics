import rospy
from mdr_head_controller.head_controller_base import HeadControllerBase


class HeadController(HeadControllerBase):
    def __init__(self):
        self.error_message = "this is only a dummy class, please use robot-specific package to control the head"
        rospy.logwarn(self.error_message)

    def look_up(self):
        rospy.logwarn(self.error_message)
        rospy.loginfo("(DUMMY) look up")
        return True

    def look_down(self):
        rospy.logwarn(self.error_message)
        rospy.loginfo("(DUMMY) look down")
        return True

    def turn_left(self):
        rospy.logwarn(self.error_message)
        rospy.loginfo("(DUMMY) turn left")
        return True

    def turn_right(self):
        rospy.logwarn(self.error_message)
        rospy.loginfo("(DUMMY) turn right")
        return True

    def reset(self):
        rospy.logwarn(self.error_message)
        rospy.loginfo("(DUMMY) reset head")
        return True

    def tilt(self, angle: float):
        rospy.logwarn(self.error_message)
        rospy.loginfo(f"(DUMMY) tilt head by '{angle}' radians")
        return True

    def pan(self, angle: float):
        rospy.logwarn(self.error_message)
        rospy.loginfo(f"(DUMMY) pan head by '{angle}' radians")
        return True
