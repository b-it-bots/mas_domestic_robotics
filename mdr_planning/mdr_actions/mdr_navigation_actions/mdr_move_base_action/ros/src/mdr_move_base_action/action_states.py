import numpy as np
import time
import rospy
import actionlib
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
import move_base_msgs.msg as move_base_msgs
from actionlib_msgs.msg import GoalStatus

from mas_tools.file_utils import load_yaml_file
from mas_tools.ros_utils import ACTIONLIB_STATUS_NAMES

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_move_base_action.msg import MoveBaseGoal, MoveBaseResult

class MoveBaseSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='/move_base',
                 pose_description_file='',
                 pose_frame='map',
                 recovery_position_m_std=0.2,
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
        self.move_base_client = None
        self.move_base_goal = move_base_msgs.MoveBaseGoal()
        self._move_base_result_state = None
        self._move_base_result = None
        self._move_base_fb = None
        self.move_base_goal_pub = None

        self.is_recovering = False
        self.recovery_count = 0
        self.recovery_position_m_std = recovery_position_m_std

    def init(self):
        self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
        rospy.loginfo('[move_base] Waiting for %s server', self.move_arm_server)
        wait_ok = self.move_arm_client.wait_for_server(timeout=rospy.Duration(self.timeout))
        if not wait_ok:
            raise ValueError("[move_base] failed to wait for '{}' action server after '{}' seconds"
                             .format(self.move_arm_server, self.timeout))

        self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, move_base_msgs.MoveBaseAction)
        wait_ok = self.move_base_client.wait_for_server(timeout=rospy.Duration(self.timeout))
        if not wait_ok:
            raise ValueError("[move_base] failed to wait for '{}' action server after '{}' seconds"
                             .format(self.move_base_server, self.timeout))

        # we also create a goal pose publisher for debugging purposes
        goal_pose_topic = self.move_base_server + '_goal'
        rospy.loginfo('[move_base] Creating a goal pose publisher on topic %s', goal_pose_topic)
        self.move_base_goal_pub = rospy.Publisher(goal_pose_topic, PoseStamped, queue_size=1)

        return FTSMTransitions.INITIALISED

    def _move_base_active_cb(self):
        rospy.loginfo("[move_base] '{}' is processing new goal".format(self.move_base_server))

    def _move_base_feedback_cb(self, feedback):
        self._move_base_fb = feedback

    def _move_base_done_cb(self, state, result):
        if state not in ACTIONLIB_STATUS_NAMES:
            rospy.logerr("[move_base] got unrecognized action goal state: {}".format(state))
        else:
            rospy.loginfo("[move_base] '{}' finished processing goal: state={}, result={}"
                          .format(self.move_base_server, ACTIONLIB_STATUS_NAMES[state], result))
        self._move_base_result = result
        self._move_base_result_state = state

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

        # we attempt a recovery in which we send the robot to a slightly
        # different pose from the one originally requested
        if self.is_recovering:
            pose.pose.position.x += np.random.normal(0., self.recovery_position_m_std)
            pose.pose.position.y += np.random.normal(0., self.recovery_position_m_std)

        self.move_base_goal.target_pose = pose
        self.move_base_goal_pub.publish(pose)

        self._move_base_fb = None
        self._move_base_result = None
        self._move_base_result_state = None
        self.move_base_client.cancel_all_goals()
        self.move_base_client.send_goal(self.move_base_goal, active_cb=self._move_base_active_cb,
                                        feedback_cb=self._move_base_feedback_cb, done_cb=self._move_base_done_cb)

        timeout_point = time.time() + self.timeout
        timed_out = False
        while self._move_base_result_state is None:
            if time.time() > timeout_point:
                rospy.logerr("[move_base] no result from '{}' after '{}' seconds"
                             .format(self.move_base_server, self.timeout))
                self.move_base_client.cancel_goal()
                timed_out = True
                break

        if self._move_base_result_state == GoalStatus.SUCCEEDED:
            rospy.loginfo('[move_base] Pose reached successfully')
            self.reset_state()
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

        if self._move_base_result_state == GoalStatus.PREEMPTED:
            rospy.logwarn("[move_base] '{}' preempted".format(self.move_base_server))

        rospy.logerr('[move_base] Pose could not be reached within %f seconds', self.timeout)
        if self.recovery_count < self.max_recovery_attempts:
            self.recovery_count += 1
            rospy.logerr('[move_base] Attempting recovery')
            return FTSMTransitions.RECOVER

        rospy.logerr("[move_base] giving up after '{}' attempts".format(self.max_recovery_attempts))
        self.reset_state()
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def recovering(self):
        self.is_recovering = True
        return FTSMTransitions.DONE_RECOVERING

    def convert_pose_name_to_coordinates(self, pose_name):
        poses = load_yaml_file(self.pose_description_file)
        try:
            coordinates = poses[pose_name]
            return coordinates
        except:
            rospy.logerr('Pose name "%s" does not exist', pose_name)
            return None

    def set_result(self, success):
        result = MoveBaseResult()
        result.success = success
        return result

    def reset_state(self):
        self.recovery_count = 0
        self.is_recovering = False
