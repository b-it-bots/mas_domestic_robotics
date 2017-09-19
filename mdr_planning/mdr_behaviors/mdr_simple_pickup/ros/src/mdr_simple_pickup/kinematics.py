import rospy
import moveit_msgs.msg
import moveit_msgs.srv
import sensor_msgs.msg
import moveit_commander


class Kinematics:

    def __init__(self, group_name):
        self.group_name = group_name
        self.commander = moveit_commander.RobotCommander()
        self.state = moveit_commander.RobotState()
        self.joint_names = self.commander.get_joint_names(self.group_name)
        
        # service clients
        rospy.loginfo('Waiting for "get_ik" service')
        rospy.wait_for_service('get_ik')
        self.ik_client = rospy.ServiceProxy('get_ik',
                                            moveit_msgs.srv.GetPositionIK)
        rospy.loginfo('Found service "get_ik"')


    def inverse_kinematics(self, goal_pose, configuration):
        '''
        Call the IK solver to calculate the joint configuration to reach the
        goal pose. The configuration parameter is the start position of the
        joints.
        
        :param goal_pose: The pose for which the inverse kinematics is solved
        :type goal_pose: geometry_msgs.msg.PoseStamped
        
        :param configuration: The initial manipulator's configuration
        :type configuration: Float[]
        
        :return: The joint configuration that reaches the requested pose
        :rtype: Float[] or None
        '''
        if (len(self.joint_names) != len(configuration)):
            return None
        
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.timeout = rospy.Duration(0.5)
        req.ik_request.attempts = 1
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ik_client(req)
        except rospy.ServiceException, e:
            rospy.logerr('Service did not process request: %s', str(e))
            return None
        
        if (resp.error_code.val == resp.error_code.SUCCESS):
            return self._extract_positions(resp.solution.joint_state, self.joint_names)
        else:
            return None


    def _extract_positions(self, joint_state, joint_names):
        '''
        :type joint_state: sensor_msgs.msg.JointState
        :type joint_names: String[]
        '''
        position = []
        
        for name in joint_names:
            for i in xrange(0, len(joint_state.name)):
                if (joint_state.name[i] == name):
                    position.append(joint_state.position[i])
        
        return position