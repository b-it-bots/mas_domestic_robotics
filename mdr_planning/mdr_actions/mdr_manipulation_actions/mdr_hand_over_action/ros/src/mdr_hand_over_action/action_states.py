#!/usr/bin/python
from os.path import join
import pickle
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_hand_over_action.msg import HandOverGoal, HandOverResult

from mdr_move_arm_action.dmp import DMPExecutor

from importlib import import_module

class HandOverSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 init_config_name = 'neutral',
                 hand_over_policy_config_dir='',
                 hand_over_position_policy_parameters_file = '800_py2.pkl',
                 hand_over_dmp_weights_dir='',
                 hand_over_dmp = 'grasp.yaml',
                 dmp_tau = 30.,
                 max_recovery_attempts=1):
        super(HandOverSM, self).__init__(
            'HandOver', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name), 'GripperController')
        self.gripper = GripperControllerClass()

        self.move_arm_server = move_arm_server

        self.init_config_name = init_config_name
        self.hand_over_policy_config_dir = hand_over_policy_config_dir
        self.hand_over_dmp_weights_dir = hand_over_dmp_weights_dir 

        self.hand_over_dmp = hand_over_dmp
        self.dmp_tau = dmp_tau
        # self.hand_over_position_policy_parameters_file = hand_over_position_policy_parameters_file
        # self.hand_over_position_policy_parameters_file = 'contextHOs_160iters-Sigma_restarts_at_10s, eps:1, N:40, Con:x-diag, Goals:(Se:[0.5, 0.078, 0.7], Ly:[0.7, 0.078, 0.7], St:[0.4, 0.078, 1.0]).pkl'
        self.hand_over_position_policy_parameters_file = join(self.hand_over_policy_config_dir, '160_py2.pkl')
        self.hand_over_position_policy_parameters = self.load_policy_params_from_file(self.hand_over_position_policy_parameters_file)

        self.tf_listener = tf.TransformListener()

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[hand_over] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[hand_over] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        return FTSMTransitions.INITIALISED

    def running(self):
        posture_type = self.goal.posture_type
        obstacle = self.goal.obstacle

        # > Pick context-dependent hand-over position:
        policy_parameter_a = self.hand_over_position_policy_parameters[0][0]
        policy_parameter_A = self.hand_over_position_policy_parameters[0][1]

        if posture_type == 'standing':
            context_vector = np.array([0.0, 0.0, 0.4])
        elif posture_type == 'seated':
            context_vector = np.array([0.5, 0.0, 0.0])
        elif posture_type == 'lying':
            context_vector = np.array([0.0, 0.7, 0.0])
        
        # Sample upper-level policy (with no exploration) for hand-over position;
        # by calculating the mean of the linear-Gaussian model, given context vector s:
        hand_over_position = np.squeeze(policy_parameter_a + context_vector.dot(policy_parameter_A))

        hand_over_pose = PoseStamped()
        hand_over_pose.header.frame_id = 'base_link'
        hand_over_pose.header.stamp = rospy.Time.now()

        hand_over_pose.pose.position.x = hand_over_position[0]
        hand_over_pose.pose.position.y = hand_over_position[1]
        hand_over_pose.pose.position.z = hand_over_position[2]

        hand_over_pose.pose.orientation.x = 0.000
        hand_over_pose.pose.orientation.y = 0.000
        hand_over_pose.pose.orientation.z = 0.000
        hand_over_pose.pose.orientation.w = 1.000

        pose_base_link = self.tf_listener.transformPose('base_link', hand_over_pose)

        ## TODO:
        # > Pick context_dependent hand-over trajectory shape:
        if not obstacle:
            trajectory_weights_filename = 'grasp.yaml'
        else:
            ## Initial learned trajectory:
            # trajectory_weights_filename = 'learned_obstacle_avoiding_weights.yaml'
            
            ## Smoothed trajectories:
            # trajectory_weights_filename = 'learned_smoothed_obstacle_avoiding_weights_2900iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_obstacle_avoiding_weights_2100iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_obstacle_avoiding_weights_1600iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_obstacle_avoiding_weights_800iters.yaml'

            ## Smoothed trajectories, with corrected end_points:
            # trajectory_weights_filename = 'learned_smoothed_corrected_obstacle_avoiding_weights_2900iters.yaml'
            trajectory_weights_filename = 'learned_smoothed_corrected_obstacle_avoiding_weights_2100iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_obstacle_avoiding_weights_1600iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_obstacle_avoiding_weights_800iters.yaml'

            ## Underhand trajectories:
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_7700iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_6900iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_6100iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_5300iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_4500iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_3700iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_2900iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_2100iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_1600iters.yaml'
            # trajectory_weights_filename = 'learned_smoothed_corrected_underhand_weights_800iters.yaml'


        self.hand_over_dmp = join(self.hand_over_dmp_weights_dir, trajectory_weights_filename)

        ## TODO: determine whether aligning base is necessary for hand_over action here:
        ## ...

        # # Go to pregrasp low position:
        # rospy.loginfo('[hand_over] Moving back to pregrasp low position...')
        # self.__move_arm(MoveArmGoal.NAMED_TARGET, 'pregrasp_low')

        # Initialize by grasping a provided object:
        # rospy.loginfo('[hand_over] Closing the gripper')
        # self.gripper.close() 

        # Move to chosen hand_over position, along appropriate trajectory:
        rospy.loginfo('[hand_over] Handing object over...')
        self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        # Wait for person to hold object:
        rospy.loginfo('[hand_over] Waiting before releasing object...')
        rospy.sleep(5.)

        # Release the object:
        rospy.loginfo('[hand_over] Opening the gripper...')
        self.gripper.open()

        # Return to a neutral arm position:
        rospy.loginfo('[hand_over] Moving back to neutral position...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.init_config_name)

        # For now, assume unconditional success:
        self.result = self.set_result(True)
        return FTSMTransitions.DONE

    def __move_arm(self, goal_type, goal):
        '''Sends a request to the 'move_arm' action server and waits for the
        results of the action execution.
        Keyword arguments:
        goal_type -- 'MoveArmGoal.NAMED_TARGET' or 'MoveArmGoal.END_EFFECTOR_POSE'
        goal -- A string if 'goal_type' is 'MoveArmGoal.NAMED_TARGET';
                a 'geometry_msgs/PoseStamped' if 'goal_type' is 'MoveArmGoal.END_EFFECTOR_POSE'
        '''
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = goal_type
        if goal_type == MoveArmGoal.NAMED_TARGET:
            move_arm_goal.named_target = goal
        elif goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            move_arm_goal.end_effector_pose = goal
            move_arm_goal.dmp_name = self.hand_over_dmp
            move_arm_goal.dmp_tau = self.dmp_tau
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

    def recovering(self):
        ## TODO: include recovery behaviours?
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = HandOverResult()
        result.success = success
        return result

    def load_policy_params_from_file(self, filename):
        with open(filename, 'rb') as f:
            return pickle.load(f)
