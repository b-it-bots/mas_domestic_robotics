import rospy

class GripperControllerBase(object):
    def open(self):
        rospy.loginfo('[OPEN_GRIPPER] Ignoring request')

    def close(self):
        rospy.loginfo('[CLOSE_GRIPPER] Ignoring request')
