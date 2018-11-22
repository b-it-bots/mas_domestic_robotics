import rospy

class GripperControllerBase(object):
    def open(self):
        rospy.loginfo('[OPEN_GRIPPER] Ignoring request')
        raise NotImplementedError()

    def close(self):
        rospy.loginfo('[CLOSE_GRIPPER] Ignoring request')
        raise NotImplementedError()

    def init_grasp_verification(self):
        rospy.loginfo('[INIT_GRASP_VERIFICATION] Ignoring request')
        raise NotImplementedError()

    def verify_grasp(self):
        rospy.loginfo('[VERIFY_GRASP] Ignoring request')
        raise NotImplementedError()
