#!/usr/bin/python
from importlib import import_module
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
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_receive_object_action.msg import ReceiveObjectResult

class ReceiveObjectSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 force_sensor_topic='/wrench/compensated',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 init_config_name = 'neutral',
                 receive_object_policy_config_dir='',
                 receive_object_position_policy_parameters_file = 'learned_position_policy_parameters.pkl',
                 receive_object_dmp = '',
                 dmp_tau = 30.,
                 person_dist_threshold = 0.5,
                 max_recovery_attempts=1):
        super(ReceiveObjectSM, self).__init__('ReceiveObject', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name), 'GripperController')
        self.gripper = GripperControllerClass()

        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server

        self.init_config_name = init_config_name
        self.receive_object_policy_config_dir = receive_object_policy_config_dir

        self.receive_object_dmp = receive_object_dmp
        self.dmp_tau = dmp_tau
        self.receive_object_position_policy_parameters_file = join(self.receive_object_policy_config_dir, receive_object_position_policy_parameters_file)
        self.receive_object_position_policy_parameters = self.load_policy_params_from_file(self.receive_object_position_policy_parameters_file)
        self.person_dist_threshold = person_dist_threshold

        self.tf_listener = tf.TransformListener()
        self.force_sensor_topic = force_sensor_topic

        self.latest_force_measurement_z = 0.
        self.cumsum_z = 0
        self.force_detection_threshold = 1.
        self.object_reception_detected = False

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[receive_object] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[receive_object] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[receive_object] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[receive_object] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        # Start with arm in neutral position:
        rospy.loginfo('[receive_object] Moving arm to neutral position...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.init_config_name)

        return FTSMTransitions.INITIALISED

    def running(self):
        receive_object_pose = PoseStamped()
        receive_object_pose.header.frame_id = 'base_link'
        if self.goal.context_aware:
            # > Pick context-dependent receive object position:
            policy_parameter_a = self.receive_object_position_policy_parameters[0][0]
            policy_parameter_A = self.receive_object_position_policy_parameters[0][1]

            if self.goal.posture_type == 'standing':
                context_vector = np.array([0.0, 0.0, 0.4])
            elif self.goal.posture_type == 'seated':
                context_vector = np.array([0.5, 0.0, 0.0])
            elif self.goal.posture_type == 'lying':
                context_vector = np.array([0.0, 0.7, 0.0])

            # Sample upper-level policy (with no exploration) for object reception position;
            # by calculating the mean of the linear-Gaussian model, given context vector s
            receive_object_position = np.squeeze(policy_parameter_a + context_vector.dot(policy_parameter_A))

            receive_object_pose.pose.position.x = receive_object_position[0]
            receive_object_pose.pose.position.y = receive_object_position[1]
            receive_object_pose.pose.position.z = receive_object_position[2]
        else:
            # Pick context-independent object reception position
            receive_object_pose.pose.position.x = 0.5
            receive_object_pose.pose.position.y = 0.078
            receive_object_pose.pose.position.z = 0.8

        receive_object_pose.pose.orientation.x = 0.000
        receive_object_pose.pose.orientation.y = 0.000
        receive_object_pose.pose.orientation.z = 0.000
        receive_object_pose.pose.orientation.w = 1.000

        self.goal.person_pose = self.tf_listener.transformPose("base_link", self.goal.person_pose)
        distance_to_person = np.sqrt(self.goal.person_pose.pose.position.x**2 + self.goal.person_pose.pose.position.y**2)
        if distance_to_person > self.person_dist_threshold:
            move_to_person_goal = MoveBaseGoal()
            move_to_person_goal.goal_type = MoveBaseGoal.POSE
            move_to_person_goal.pose.header.frame_id = self.goal.person_pose.header.frame_id
            move_to_person_goal.pose.pose.position.x = self.goal.person_pose.pose.position.x - 0.3
            move_to_person_goal.pose.pose.position.y = self.goal.person_pose.pose.position.y - 0.3
            rospy.loginfo('[receive_object] Moving to person...')

            self.move_base_client.send_goal(move_to_person_goal)

            # we try moving towards the person before receiving the object;
            # the person should be close by, so we move for a short time
            # since the robot will either reach the person in that time,
            # or will stop as close to the person as possible
            self.move_base_client.wait_for_result(rospy.Duration(10.))
            self.move_base_client.cancel_goal()

        # Start with arm in neutral position:
        rospy.loginfo('[receive_object] Moving arm to neutral position...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.init_config_name)

        # Move to chosen receive_object position, along appropriate trajectory:
        rospy.loginfo('[receive_object] Receiving object...')
        self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, receive_object_pose)

        if not self.goal.reception_detection:
            # Naive object release strategy:
            rospy.loginfo('[receive_object] Waiting before releasing object...')
            rospy.sleep(5.)

            # Release the object:
            rospy.loginfo('[receive_object] Opening the gripper...')
            self.gripper.open()

            # Since no detection is used here, the object is released and we always set the reception to True.
            self.object_reception_detected = True
        else:
            # Force sensing object release strategy:
            rospy.sleep(1.)
            rospy.loginfo('[receive_object] Waiting for object to be received...')
            rospy.Subscriber(self.force_sensor_topic, WrenchStamped, self.force_sensor_cb)
            self.object_reception_detected = False
            self.cumsum_z = 0.

            start_time = rospy.get_time()
            while (rospy.get_time() - start_time) < 50 and not self.object_reception_detected:
                self.detect_object_reception()
                rospy.sleep(0.1)

            # If a pull is detected, release the object:
            if self.object_reception_detected:
                rospy.loginfo('[receive_object] Closing the gripper to receive the object...')
                self.gripper.close()
            else:
                rospy.loginfo('[receive_object] Not waiting anymore since no reception was detected')

        # Return to a neutral arm position:
        rospy.loginfo('[receive_object] Moving back to neutral position...')
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
            move_arm_goal.dmp_name = self.receive_object_dmp
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
        result = ReceiveObjectResult()
        result.success = success
        return result

    def load_policy_params_from_file(self, filename):
        with open(filename, 'rb') as f:
            return pickle.load(f)

    def detect_object_reception(self):
        mu_0 = 0.
        mu_1 = -6.
        std = 1.

        pdf_0 = norm(mu_0, std)
        pdf_1 = norm(mu_1, std)

        self.cumsum_z += max(0., np.log(pdf_1.pdf(self.latest_force_measurement_z) / pdf_0.pdf(self.latest_force_measurement_z)))

        if self.cumsum_z > self.force_detection_threshold:
            rospy.loginfo('[receive_object] Object reception detected!')
            self.object_reception_detected = True

    def force_sensor_cb(self, force_sensor_msg):
        self.latest_force_measurement_z = force_sensor_msg.wrench.force.z
