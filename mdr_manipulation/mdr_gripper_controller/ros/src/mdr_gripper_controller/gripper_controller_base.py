import rospy

class GripperControllerBase(object):
    def open(self):
        rospy.loginfo('[OPEN_GRIPPER] Ignoring request')
        return True

    def close(self):
        rospy.loginfo('[CLOSE_GRIPPER] Ignoring request')
        return True

    def verify_grasp(self):
        rospy.loginfo('[VERIFY_GRASP] Ignoring request')
        return True
