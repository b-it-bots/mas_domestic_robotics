#!/usr/bin/python
from os.path import join
import pickle
import numpy as np
from scipy.stats import norm

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_hand_over_action.msg import HandOverGoal, HandOverResult

from mdr_move_arm_action.dmp import DMPExecutor

from importlib import import_module

class HandOverSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 force_sensor_topic='/hsrb/wrist_wrench/raw',
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
        self.force_sensor_topic = force_sensor_topic

        self.latest_force_measurement_x = 0.
        self.latest_force_measurement_z = 0.
        self.cumsum_x = 0
        self.cumsum_z = 0
        self.force_detection_threshold = 1.
        self.object_reception_detected = False

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
        context_aware = self.goal.context_aware

        hand_over_pose = PoseStamped()
        hand_over_pose.header.frame_id = 'base_link'
        hand_over_pose.header.stamp = rospy.Time.now()
        if context_aware:
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

            hand_over_pose.pose.position.x = hand_over_position[0]
            hand_over_pose.pose.position.y = hand_over_position[1]
            hand_over_pose.pose.position.z = hand_over_position[2]
        else:
            # Pick context-independent hand-over position:
            hand_over_pose.pose.position.x = 0.5
            hand_over_pose.pose.position.y = 0.078
            hand_over_pose.pose.position.z = 0.8

        hand_over_pose.pose.orientation.x = 0.000
        hand_over_pose.pose.orientation.y = 0.000
        hand_over_pose.pose.orientation.z = 0.000
        hand_over_pose.pose.orientation.w = 1.000

        pose_base_link = self.tf_listener.transformPose('base_link', hand_over_pose)

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

        # # Initialize by grasping a provided object:
        # rospy.loginfo('[hand_over] Closing the gripper')
        # self.gripper.close() 

        # Move to chosen hand_over position, along appropriate trajectory:
        rospy.loginfo('[hand_over] Handing object over...')
        self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        # Naive object release strategy:
        # # --------------------------------------------------------------
        # # Wait for person to hold object:
        # rospy.loginfo('[hand_over] Waiting before releasing object...')
        # rospy.sleep(5.)

        # # Release the object:
        # rospy.loginfo('[hand_over] Opening the gripper...')
        # self.gripper.open()
        # #  --------------------------------------------------------------

        # Force sensing object release strategy:
        # --------------------------------------------------------------
        # Wait for person to pull object:
        rospy.sleep(1.)
        rospy.loginfo('[hand_over] Waiting for object to be received...')
        rospy.Subscriber(self.force_sensor_topic, WrenchStamped, self.force_sensor_cb)
        self.object_reception_detected = False
        self.cumsum_x = 0.
        self.cumsum_z = 0.

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < 50 and not self.object_reception_detected:
            self.detect_object_reception()
            rospy.sleep(0.1)

        # If a pull is detected, release the object:
        if self.object_reception_detected:        
            rospy.loginfo('[hand_over] Opening the gripper to release object...')
            self.gripper.open()
        else:
            rospy.loginfo('[hand_over] Keeping object since no reception was detected')
        # --------------------------------------------------------------        

        # Return to a neutral arm position:
        rospy.loginfo('[hand_over] Moving back to neutral position...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.init_config_name)

        if self.object_reception_detected:
            self.result = self.set_result(True)
        else:
            self.result = self.set_result(False)
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

    def detect_object_reception(self):
        mu_0 = -2
        mu_1 = -20
        std = 1

        pdf_0 = norm(mu_0, std)
        pdf_1 = norm(mu_1, std)

        self.cumsum_x += max(0, np.log(pdf_1.pdf(self.latest_force_measurement_x) / pdf_0.pdf(self.latest_force_measurement_x)))
        self.cumsum_z += max(0, np.log(pdf_1.pdf(self.latest_force_measurement_z) / pdf_0.pdf(self.latest_force_measurement_z)))

        print(self.cumsum_x)
        if self.cumsum_x > self.force_detection_threshold:
        # if self.cumsum_x > self.force_detection_threshold or self.cumsum_z > self.force_detection_threshold:
            rospy.loginfo('[hand_over] Object reception detected!')
            self.object_reception_detected = True
        
    def force_sensor_cb(self, force_sensor_msg):
        self.latest_force_measurement_x = force_sensor_msg.wrench.force.x
        self.latest_force_measurement_z = force_sensor_msg.wrench.force.z

        # print('[hand_over DEBUG] Force Measurements:')
        # print('[hand_over DEBUG] Current cumsum in x:', self.cumsum_x)
        # print('[hand_over DEBUG] Current cumsum in z:', self.cumsum_z)
        # print("*********************")
        # print('[hand_over DEBUG] Current Force Measurements, x:', force_sensor_msg.wrench.force.x)
        # print('[hand_over DEBUG] Current Force Measurements, y:', force_sensor_msg.wrench.force.y)
        # print('[hand_over DEBUG] Current Force Measurements, z:', force_sensor_msg.wrench.force.z)
        # print("\n\n")
