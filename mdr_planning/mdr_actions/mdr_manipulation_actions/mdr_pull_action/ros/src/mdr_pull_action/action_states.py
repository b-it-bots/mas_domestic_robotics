#!/usr/bin/python
import rospkg
from importlib import import_module
import numpy as np

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
from mdr_pull_action.msg import PullGoal, PullFeedback, PullResult

from geometry_msgs.msg import WrenchStamped
from scipy.fftpack import fft
from scipy.stats import norm

class PullSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=0.078,
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=1,
                 max_recovery_attempts=1):
        super(PullSM, self).__init__('Pull', [], max_recovery_attempts)
        self.grasped_limits = np.load(rospkg.RosPack().get_path('mdr_pull_action')+"/ros/scripts/grasped_limits.npy")
        self.empty_limits = np.load(rospkg.RosPack().get_path('mdr_pull_action')+"/ros/scripts/empty_limits.npy")

        self.timeout = timeout

        self.initial_pose = PullGoal()
        self.initial_pose.pose.header.frame_id = 'base_link'
        self.initial_pose.pose.header.stamp = rospy.Time.now()
        self.initial_pose.pose.pose.position.x = 0.62
        self.initial_pose.pose.pose.position.y = base_elbow_offset
        self.initial_pose.pose.pose.position.z = 0.8
        self.initial_pose.pose.pose.orientation.x = 0.758
        self.initial_pose.pose.pose.orientation.z = 0.652

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server
        self.base_elbow_offset = base_elbow_offset
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau
        self.number_of_retries = number_of_retries

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None
        
        self.speech_pub = rospy.Publisher('/say',String,queue_size = 1)
        rospy.Subscriber('hsrb/wrist_wrench/raw', WrenchStamped, self.wrist_force_cb, queue_size = 1)
        self.record_wrist_force_vals = False
        self.execute_fdd = False
        self.wrist_force_list = []
        self.sliding_window = np.array([])
        self.window_size = 50
        
        self.wrist_sig_std = 0.004
        self.wrist_freq_std = 0.02
        self.wrist_sig_mean = 0.0
        self.wrist_freq_mean = 0.0
        
        self.grasped_state = False
        self.empty_state = False

    def distribution(self,occurrence, bins):
        probabilities = []
        bin_size = (bins[1]-bins[0])/6
        init_val = bins[0]
        for i in range(5):
            prob = bin_size*occurrence[i]
            probabilities.append(prob)
        probabilities.append(1-np.sum(probabilities))
        rdm_num = np.random.choice(range(6), p=probabilities)
        chosen_num = np.random.uniform(init_val+(bin_size*rdm_num),init_val+(bin_size*(rdm_num+1)))
        return chosen_num
   
    def get_mean_freq_and_sig(self,signal):
        freq_list = []
        sig_list = []
        N = 50
        for x in range(len(signal)-(N-1)):
            yf = fft(signal[x:(x+N)])
            yf_abs = 2.0/N * np.abs(yf[0:N//2])
            freq_list.append(yf_abs[0]+yf_abs[1])
            sig_list.append(np.mean(signal[x:(x+N)]))
        return np.mean(sig_list), np.mean(freq_list) 

 
    def wrist_force_cb(self,msg):
        if self.record_wrist_force_vals:
            self.wrist_force_list.append(msg.wrench.torque.y)
        if self.execute_fdd:
            if len(self.wrist_force_list) < 50:
                self.wrist_force_list.append(msg.wrench.torque.y)
            else:
                self.wrist_force_list.pop(0)
                self.wrist_force_list.append(msg.wrench.torque.y)
                mean, freq = self.get_mean_freq_and_sig(self.wrist_force_list)
    
                likelihood = np.log(norm.pdf(freq, loc = self.wrist_freq_mean, scale = self.wrist_freq_std)) + np.log(norm.pdf(mean, loc = self.wrist_sig_mean, scale = self.wrist_sig_std))
                if len(self.sliding_window) < self.window_size:
                    self.sliding_window = np.array([0] + list(self.sliding_window))
                    self.sliding_window = self.sliding_window + likelihood
                else:
                    self.sliding_window[1:] = self.sliding_window[:-1] + likelihood
                    self.sliding_window[0] = likelihood
                sliding_window_len = len(self.sliding_window)
                if (self.grasped_state == False and len(np.where(self.sliding_window < self.empty_limits[:sliding_window_len])[0])>0) and self.empty_state == False: 
                    self.grasped_state = True
                    self.speech_pub.publish("Object Grasped")
                if self.empty_state == False and len(np.where(self.sliding_window > self.grasped_limits[:sliding_window_len])[0])>0: 
                    self.empty_state = True
                    if self.grasped_state == True:
                        self.speech_pub.publish("Lost Grip of Object")
                        self.grip_lost_time = rospy.get_time()
                    else:
                        self.speech_pub.publish("Grasp Failed")

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
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        action_status = 'fail'
        retry_count = 0
        while (not action_status=='success') and (retry_count <= self.number_of_retries):
            if retry_count > 0:
                rospy.loginfo('[pickup] Retrying grasp')
                if action_status == 'grasp_fail':
                    self.speech_pub.publish("Re trying grasp")
                    x_0 = np.array([32.06432054, 16.03216027, 40.08040067, 0, 0, 8.01608013])
                    x_1 = np.array([0.43498565, 0.49736027])
                    y_0 = np.array([13.51563889, 27.03127777, 0, 27.03127777, 54.06255555, 40.54691666])
                    y_1 = np.array([0.05675844, 0.09375263])
                    z_0 = np.array([109.2043287, 27.30108217, 27.30108217, 54.60216435, 81.90324652, 27.30108217])
                    z_1 = np.array([0.80388078, 0.82219507])
    
                    pose_base_link.pose.position.x = self.distribution(x_0, x_1)
                    pose_base_link.pose.position.y = self.distribution(y_0, y_1)
                    pose_base_link.pose.position.z = self.distribution(z_0, z_1)
                else :    #i.e action_status == 'grasp_lost'
                    self.speech_pub.publish("Trying to grasp lost object")
                    pose_base_link.pose.position.x = (self.base_motion_end_time - self.grip_lost_time)*0.1 + 0.25
        
            goal_str = "Attempting to pick object at: x=%.3f,y=%.3f,z=%.3f" % (pose_base_link.pose.position.x, pose_base_link.pose.position.y, pose_base_link.pose.position.z)
            rospy.loginfo(goal_str)
            
            base_motion_y = 0
            if pose_base_link.pose.position.y != self.base_elbow_offset:
                base_motion_y = pose_base_link.pose.position.y - self.base_elbow_offset 
                self.__align_base_with_pose(pose_base_link)
                pose_base_link.pose.position.y = self.base_elbow_offset

            rospy.loginfo('[pickup] Opening the gripper...')

            self.initial_pose.pose.pose.position.z = pose_base_link.pose.position.z
            self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, self.initial_pose.pose)
            rospy.loginfo('[pickup] Grasping...')
            
            rospy.sleep(0.5)
            self.gripper.close()
            rospy.sleep(0.5)
            self.record_wrist_force_vals = True 
            rospy.sleep(2)
            self.record_wrist_force_vals = False    
            self.wrist_sig_mean, self.wrist_freq_mean = self.get_mean_freq_and_sig(self.wrist_force_list)
            self.wrist_force_list = []
            self.gripper.open()
            
            dist = pose_base_link.pose.position.x
            self.__move_base_along_x(dist)
            rospy.sleep(0.5)
            rospy.loginfo('[pickup] Arm motion successful')

            rospy.loginfo('[pickup] Closing the gripper')
            self.gripper.close()
            rospy.sleep(1.5)
            self.execute_fdd = True
            self.__move_base_along_x(-(dist-0.1))
            self.base_motion_end_time = rospy.get_time()
            rospy.sleep(1)
            self.execute_fdd = False
            if self.grasped_state == True and self.empty_state == False:
                self.speech_pub.publish("Pull Action Complete")
                action_status = 'success'
            else:
                retry_count += 1
                self.speech_pub.publish("Pull Action Failed")
                if self.grasped_state == False:
                    action_status = 'grasp_fail'
                else:
                    action_status = 'grasp_lost'

            self.empty_state = False
            self.grasped_state = False
            self.wrist_force_list = []
            self.sliding_window = np.array([])

            self.gripper.open()
            rospy.sleep(0.5)

            rospy.loginfo('[pickup] Moving the base back to the original position')
            self.__move_base_along_x(-0.1)

            rospy.loginfo('[pickup] Moving the arm back')

            pose_base_link.pose.position.y = self.base_elbow_offset - base_motion_y
            self.__align_base_with_pose(pose_base_link)
            pose_base_link.pose.position.y = self.base_elbow_offset + base_motion_y

        if action_status == 'success':
            self.result = self.set_result(True)
            return FTSMTransitions.DONE
    
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def __align_base_with_pose(self, pose_base_link):
        '''Moves the base so that the elbow is aligned with the goal pose.

        Keyword arguments:
        pose_base_link -- a 'geometry_msgs/PoseStamped' message representing
                          the goal pose in the base link frame

        '''
        aligned_base_pose = PoseStamped()
        aligned_base_pose.header.frame_id = 'base_link'
        aligned_base_pose.header.stamp = rospy.Time.now()
        aligned_base_pose.pose.position.x = 0.
        aligned_base_pose.pose.position.y = pose_base_link.pose.position.y - self.base_elbow_offset
        aligned_base_pose.pose.position.z = 0.
        aligned_base_pose.pose.orientation.x = 0.
        aligned_base_pose.pose.orientation.y = 0.
        aligned_base_pose.pose.orientation.z = 0.
        aligned_base_pose.pose.orientation.w = 1.

        move_base_goal = MoveBaseGoal()
        move_base_goal.goal_type = MoveBaseGoal.POSE
        move_base_goal.pose = aligned_base_pose
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()
        self.move_base_client.get_result()

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

    def set_result(self, success):
        result = PullResult()
        result.success = success
        return result

