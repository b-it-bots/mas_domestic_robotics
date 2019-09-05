#!/usr/bin/python
from importlib import import_module
import numpy as np
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from skimage import measure
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import String

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_push_action.msg import PushGoal, PushFeedback, PushResult

class PushSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=0,
                 max_recovery_attempts=1):
        super(PushSM, self).__init__('Push', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.force_sensor_topic='/hsrb/wrist_wrench/raw'
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau
        self.number_of_retries = number_of_retries

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None

        self.take_image = False
        
        #rospy.Subscriber(self.force_sensor_topic, WrenchStamped, self.force_sensor_cb)
        image_topic = '/hsrb/hand_camera/image_raw'
        # Set up your subscriber and define its callback
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)
    

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_arm_server)

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_base_server)

        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_forward_server)

        return FTSMTransitions.INITIALISED

    def running(self):
        grasp_successful = False
        retry_count = 0
        
        while (not grasp_successful) and (retry_count <= self.number_of_retries):
            if retry_count > 0:
                rospy.loginfo('[push] Retrying grasp')

            #rospy.loginfo('[push] Opening the gripper...')
            self.gripper.open()

            self.pose1 = PushGoal()
            self.pose1.pose.header.frame_id = 'base_link'
            self.pose1.pose.header.stamp = rospy.Time.now()
            self.pose1.pose.pose.position.x = 0.62
            self.pose1.pose.pose.position.y = 0.078
            self.pose1.pose.pose.position.z = 0.84+(np.random.random()/50)
            self.pose1.pose.pose.orientation.x = 0.758
            self.pose1.pose.pose.orientation.y = 0.000
            self.pose1.pose.pose.orientation.z = 0.652
            self.pose1.pose.pose.orientation.w = 0.000

            self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, self.pose1.pose)
            
            self.gripper.close()

            #Push
            #self.__move_base_along_x(0.05)
            
            #self.gripper.open()
            
            #self.__move_arm(MoveArmGoal.NAMED_TARGET, 'neutral')           
            #self.__move_base_along_x(-0.05)
            grasp_successful = self.gripper.verify_grasp()
            #if grasp_successful:
                #rospy.loginfo('[push] Successfully grasped object')
            #else:
                #rospy.loginfo('[push] Grasp unsuccessful')
                #retry_count += 1
	    
        self.take_image = True
 		
        if grasp_successful:
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

        #rospy.loginfo('[push] Grasp could not be performed successfully')
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
            move_arm_goal.dmp_name = self.grasping_dmp
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

    def force_sensor_cb(self, force_sensor_msg):
        '''
        Force sensor callback. Taken from hand_over action.
        '''
        print(force_sensor_msg.wrench.force.x, force_sensor_msg.wrench.force.y, force_sensor_msg.wrench.force.z)

    def image_callback(self, msg):
        #taken from https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
        if self.take_image == True:
            bridge = CvBridge()
            print("Received an image!")
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                # Save your OpenCV2 image as a jpeg 
                print("Comparing images")
                #value of structural similarity of image where value lies between -1 to 1 and 1 is perfect match
                val_ssim= self.compare_images(cv2_img)
                
                if val_ssim > 0.40:
                    print("Grasp successful")
                else :
                    print("Grasp failed") 

            self.take_image = False

    def compare_images(self,imageB):

        # compute the mean squared error and structural similarity
        # index for the images
        imageA = cv2.imread("grasp_model4.jpeg")
        gray1 = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
        similarity_of_structure = measure.compare_ssim(gray1, gray2)
 
        return similarity_of_structure 


    def set_result(self, success):
        result = PushResult()
        result.success = success
        return result
