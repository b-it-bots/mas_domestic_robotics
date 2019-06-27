#!/usr/bin/python
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_handle_open_action.msg import HandleOpenGoal, HandleOpenResult

from mdr_move_arm_action.dmp import DMPExecutor

from importlib import import_module

#--------------------------
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np



#-----------------------------------------------

class HandleOpenSM(ActionSMBase):
    ## TODO: determine any other needed parameters
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 pregrasp_config_name='pregrasp_low',
                 final_config_name = 'neutral',
                 handle_open_dmp = '',
                 dmp_tau = 30.,
                 max_recovery_attempts=1):
        super(HandleOpenSM, self).__init__(
            'HandleOpen', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name), 'GripperController')
        self.gripper = GripperControllerClass()

        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server

        self.pregrasp_config_name = pregrasp_config_name
        self.final_config_name = final_config_name

        self.handle_open_dmp = handle_open_dmp
        self.dmp_tau = dmp_tau

        self.tf_listener = tf.TransformListener()

	self.Image_number = 0
	self.Last_image = None
	# Instantiate CvBridge
	self.bridge = CvBridge()

	self.failure_data = []


    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[handle_open] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[handle_open] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
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

        image_topic = "/hsrb/hand_camera/image_raw"
        # Set up your subscriber and define its callback
	self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)
 

        # Moving robot base back (instead of moving arm back):
        rospy.loginfo('[handle_open] Moving the base back')
        ## TODO: define backward_movement_distance, a distance value (in m?)
        self.__move_base_along_x(-0.3)

        # New: grasp verification
        # rospy.loginfo('[handle_open] Verifying whether handle was grasped...')
        # grasp_successful = self.gripper.verify_grasp()
        #if grasp_successful:
        #    rospy.loginfo('[handle_open] Successfully grasped object')
        #else:
        #    rospy.loginfo('[handle_open] Grasp unsuccessful')
     	self.sub.unregister()

	mean = np.mean(np.array(self.failure_data))
	print("======> MEAN :",mean)
        rospy.sleep(5)
        rospy.loginfo('[handle_open] Opening the gripper...')
        self.gripper.open()

        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.final_config_name)

        # For now, assume success:
        self.result = self.set_result(True)
        return FTSMTransitions.DONE

    def __prepare_handle_grasp(self, pose_base_link):
        rospy.loginfo('[handle_open] Moving to a pregrasp configuration...')
        # ...
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)

        rospy.loginfo('[handle_open] Opening the gripper...')
        self.gripper.open()
            
        # New: grasp verification initilisation
        # rospy.loginfo('[pickup] Preparing for grasp verification')
        # self.gripper.init_grasp_verification()

        ## TODO: Implement wrist rotation
        rospy.loginfo('[handle_open] Rotating the gripper...')
        self.gripper.rotate_wrist(np.pi/2.)        

        rospy.loginfo('[handle_open] Moving to intermediate grasping pose...')
        ## TODO: Move arm a specified distance after going to pregrasp_low, if needed:
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
        self.move_forward_client.wait_for_result()
        self.move_forward_client.get_result()

    def recovering(self):
        ## TODO: if recovery behaviours are appropriate, fill this method with
        ## the recovery logic
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = HandleOpenResult()
        result.success = success
        return result


    def image_callback(self,msg):
        print("Received an image!")
        self.Image_number += 1	
        try:
        # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
	    if self.Last_image is not None :
                diff_img = cv2.absdiff(self.Last_image,cv2_img)
                black_percentage = (len(np.where(diff_img == 0)[0])/float(640*480*3))
		print(diff_img.shape,black_percentage)
		self.failure_data.append(black_percentage)
	        if black_percentage > 0.3 :
                    print(black_percentage," ----> FAILURE!!!")
		else:
		    print(black_percentage," ----> SUCCESS!!!")

            self.Last_image = cv2_img

        except :
            pass

        else:
        # Save your OpenCV2 image as a jpeg 
        #cv2.imwrite('Images/camera_image_'+str(Image_number)+'.jpeg', cv2_img)
	    pass


