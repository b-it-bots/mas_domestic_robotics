import yaml
import rospy
import actionlib
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
import move_base_msgs.msg as move_base_msgs

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_move_base_action.msg import MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

class MoveBaseSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='/move_base',
                 pose_description_file='',
                 pose_frame='map',
                 max_recovery_attempts=1):
        super(MoveBaseSM, self).__init__('MoveBase', [], max_recovery_attempts)
        self.pose = None
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.pose_description_file = pose_description_file
        self.pose_frame = pose_frame
        self.timeout = timeout
        self.move_arm_client = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[move_base] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[move_base] %s server does not seem to respond', self.move_arm_server)
        return FTSMTransitions.INITIALISED

    def running(self):
        rospy.loginfo('[move_base] Moving the arm to a safe configuration...')
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        move_arm_goal.named_target = self.safe_arm_joint_config
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()

        pose = PoseStamped()
        if self.goal.goal_type == MoveBaseGoal.NAMED_TARGET:
            destination = self.goal.destination_location
            rospy.loginfo('[move_base] Moving base to %s', destination)

            self.pose = self.convert_pose_name_to_coordinates(destination)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.pose_frame
            pose.pose.position.x = self.pose[0]
            pose.pose.position.y = self.pose[1]

            quat = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
            pose.pose.orientation = Quaternion(*quat)
        elif self.goal.goal_type == MoveBaseGoal.POSE:
            pose = self.goal.pose
            rospy.loginfo('[move_base] Moving base to %s', pose)
        else:
            rospy.logerr('[move_base] Received an unknown goal type; ignoring request')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        goal = move_base_msgs.MoveBaseGoal()
        goal.target_pose = pose

        move_base_client = actionlib.SimpleActionClient(self.move_base_server,
                                                        move_base_msgs.MoveBaseAction)
        move_base_client.wait_for_server()
        move_base_client.send_goal(goal)
        success = move_base_client.wait_for_result()

        if success:
            rospy.loginfo('[move_base] Pose reached successfully')
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

        rospy.logerr('[move_base] Pose could not be reached')
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def convert_pose_name_to_coordinates(self, pose_name):
        stream = open(self.pose_description_file, 'r')
        poses = yaml.load(stream)
        stream.close()
        try:
            coordinates = poses[pose_name]
            return coordinates
        except:
            rospy.logerr('Pose name "%s" does not exist' % (pose_name))
            return None

    def set_result(self, success):
        result = MoveBaseResult()
        result.success = success
        return result
