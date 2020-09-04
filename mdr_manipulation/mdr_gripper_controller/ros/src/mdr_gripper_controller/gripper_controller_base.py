import rospy

class GripperControllerBase(object):
    def open(self):
        rospy.loginfo('[open] Ignoring request')
        raise NotImplementedError()

    def close(self):
        rospy.loginfo('[close] Ignoring request')
        raise NotImplementedError()

    def init_impact_detection_z(self):
        rospy.loginfo('[init_impact_detection_z] Ignoring request')
        raise NotImplementedError()

    def detect_impact_z(self):
        rospy.loginfo('[detect_impact_z] Ignoring request')
        raise NotImplementedError()

    def init_grasp_verification(self):
        rospy.loginfo('[init_grasp_verification] Ignoring request')
        raise NotImplementedError()

    def verify_grasp(self):
        rospy.loginfo('[verify_grasp] Ignoring request')
        raise NotImplementedError()
