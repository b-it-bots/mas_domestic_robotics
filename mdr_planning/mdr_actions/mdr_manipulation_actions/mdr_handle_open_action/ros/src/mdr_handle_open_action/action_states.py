#!/usr/bin/python
from importlib import import_module
import numpy as np
from scipy.stats import norm
import cv2
from cv_bridge import CvBridge

import rospy
import tf
import actionlib
from geometry_msgs.msg import WrenchStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_handle_open_action.msg import HandleOpenResult

class HandleOpenSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_forward_server='move_forward_server',
                 force_sensor_topic='/force_wrist/raw',
                 pregrasp_config_name='pregrasp_low',
                 final_config_name='pregrasp',
                 handle_open_dmp='',
                 dmp_tau=30.,
                 max_recovery_attempts=1):
        super(HandleOpenSM, self).__init__('HandleOpen', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name), 'GripperController')
        self.gripper = GripperControllerClass()

        self.move_arm_server = move_arm_server
        self.move_forward_server = move_forward_server

        self.pregrasp_config_name = pregrasp_config_name
        self.final_config_name = final_config_name

        self.handle_open_dmp = handle_open_dmp
        self.dmp_tau = dmp_tau

        self.tf_listener = tf.TransformListener()

        self.force_sensor_topic = force_sensor_topic

        self.latest_force_measurement_x = 0.
        self.cumsum_x = 0
        self.force_detection_threshold = 15.
        self.handle_slip_detected = False

        # For Perception-based failure detection
        self.image_number = 0
        self.last_image = None
        self.bridge = CvBridge()
        self.failure_data = []

        self.move_arm_client = None
        self.move_forward_client = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[handle_open] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[handle_open] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[handle_open] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        return FTSMTransitions.INITIALISED

    def running(self):
        pose = self.goal.handle_pose
        # string handle_type = self.goal.handle_type
        # init_end_effector_pose = self.goal.init_end_effector_pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        ## TODO: determine whether this step is necessary for handle_open action:
        # if self.base_elbow_offset > 0:
        #     self.__align_base_with_pose(pose_base_link)

        #     # the base is now correctly aligned with the pose, so we set the
        #     # y position of the goal pose to the elbow offset
        #     pose_base_link.pose.position.y = self.base_elbow_offset

        rospy.loginfo('[handle_open] Preparing grasping pose')
        pose_base_link = self.__prepare_handle_grasp(pose_base_link)

        rospy.loginfo('[handle_open] Grasping...')
        arm_motion_success = self.__move_arm(
            MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        if not arm_motion_success:
            rospy.logerr('[handle_open] Arm motion unsuccessful')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        rospy.loginfo('[handle_open] Arm motion successful')
        rospy.loginfo('[handle_open] Closing the gripper')
        self.gripper.close()

        # Moving robot base back (instead of moving arm back):
        rospy.loginfo('[handle_open] Moving the base back')
        ## TODO: define backward_movement_distance
        self.__move_base_along_x(-0.3)

        # Force sensing grasp monitoring strategy:
        # --------------------------------------------------------------
        rospy.loginfo('[handle_open] Monitoring grasp on handle...')
        self.force_sub = rospy.Subscriber(self.force_sensor_topic, WrenchStamped, self.force_sensor_cb)
        self.cumsum_x = 0.

        handle_slip_detected = False
        while not self.move_forward_client.get_result() and not handle_slip_detected:
            handle_slip_detected = self.detect_handle_slip()
            rospy.sleep(1)

        if handle_slip_detected:
            rospy.loginfo('[handle_open] Failure detected!')
            self.result = self.set_result(False)
            ## TODO: Implement recovery; abort move base, re-try, etc.
        else:
            rospy.loginfo('[handle_open] No failure detected')
            self.result = self.set_result(True)

        # TODO: Perception based grasp monitoring strategy:
        # --------------------------------------------------------------
        # image_topic = "/hsrb/hand_camera/image_raw"
        # self.sub = rospy.Subscriber(image_topic, Image, self.image_cb)

        rospy.loginfo('[handle_open] Opening the gripper...')
        self.gripper.open()

        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.final_config_name)

        return FTSMTransitions.DONE

    def __prepare_handle_grasp(self, pose_base_link):
        rospy.loginfo('[handle_open] Moving to a pregrasp configuration...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)

        rospy.loginfo('[handle_open] Opening the gripper...')
        self.gripper.open()

        rospy.loginfo('[handle_open] Rotating the gripper...')
        self.gripper.rotate_wrist(np.pi/2.)

        rospy.loginfo('[handle_open] Moving to intermediate grasping pose...')
        # Move arm a specified distance after going to pregrasp_low, if needed:
        # pose_base_link.pose.position.x -= 0.02
        # pose_base_link.pose.position.z += 0.02
        self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        return pose_base_link

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
            move_arm_goal.dmp_name = self.handle_open_dmp
            move_arm_goal.dmp_tau = self.dmp_tau
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

    def __move_base_along_x(self, distance_to_move):
        movement_speed = np.sign(distance_to_move) * 0.1 # m/s
        movement_duration = distance_to_move / movement_speed
        move_forward_goal = MoveForwardGoal()
        move_forward_goal.movement_duration = movement_duration
        move_forward_goal.speed = movement_speed
        self.move_forward_client.send_goal(move_forward_goal)
        # self.move_forward_client.wait_for_result()
        # self.move_forward_client.get_result()

    def recovering(self):
        ## TODO: implement any recovery behaviours here
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = HandleOpenResult()
        result.success = success
        return result

    def detect_handle_slip(self):
        '''
        Detecting change in force readings using CUSUM.
        '''
        mu_0 = -12
        mu_1 = -15
        std = 1.5

        pdf_0 = norm(mu_0, std)
        pdf_1 = norm(mu_1, std)

        self.cumsum_x += max(0, np.log(pdf_1.pdf(self.latest_force_measurement_x) / pdf_0.pdf(self.latest_force_measurement_x)))

        if self.cumsum_x > self.force_detection_threshold:
            rospy.loginfo('[handle_open] Handle slip detected!')
            return True
        return False

    def force_sensor_cb(self, force_sensor_msg):
        '''
        Force sensor callback. Taken from hand_over action.
        '''
        self.latest_force_measurement_x = force_sensor_msg.wrench.force.x

        # print('[hand_over DEBUG] Force Measurements:')
        # print('[hand_over DEBUG] Current cumsum in x:', self.cumsum_x)
        # print("*********************")
        # print('[hand_over DEBUG] Current Force Measurements, x:', force_sensor_msg.wrench.force.x)
        # print('[hand_over DEBUG] Current Force Measurements, y:', force_sensor_msg.wrench.force.y)
        # print('[hand_over DEBUG] Current Force Measurements, z:', force_sensor_msg.wrench.force.z)
        # print('{0}\t{1}\t{2}'.format(force_sensor_msg.wrench.force.x, force_sensor_msg.wrench.force.y, force_sensor_msg.wrench.force.z))
        # print("\n\n")

    def image_cb(self,msg):
        '''
        Callback for receiving wrist camera image
        '''
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.last_image is not None:
                diff_img = cv2.absdiff(self.last_image, cv2_img)
                black_percentage = (len(np.where(diff_img == 0)[0])/float(640*480*3))
            self.failure_data.append(black_percentage)

            if black_percentage > 0.3:
                rospy.loginfo("[handle_open] Handle slip detected!")
            self.last_image = cv2_img
        except Exception as e:
            # Exception when the image is 'None'
            rospy.loginfo("Error '{0}' occured".format(e.message))
