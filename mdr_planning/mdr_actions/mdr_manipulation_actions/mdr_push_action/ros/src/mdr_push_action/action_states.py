#!/usr/bin/python
import rospy
import tf
import actionlib
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_push_action.msg import PushGoal, PushResult
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from importlib import import_module

class PushSM(ActionSMBase):
    def __init__(self, timeout=120.0, max_recovery_attempts=1,
                gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=-1.,
                 arm_base_offset=-1.,
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=0,
                 safe_arm_joint_config='folded',
                 grasping_orientation=list()):
        super(PushSM, self).__init__('Push', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server
        self.base_elbow_offset = base_elbow_offset
        self.arm_base_offset = arm_base_offset
        self.grasping_orientation = grasping_orientation
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau
        self.number_of_retries = number_of_retries

        self.tf_listener = tf.TransformListener()

        

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None


    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[push] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[push] %s server does not seem to respond', self.move_arm_server)
            return FTSMTransitions.INIT_FAILED

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[push] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[push] %s server does not seem to respond', self.move_base_server)
            return FTSMTransitions.INIT_FAILED
        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[push] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except:
            rospy.logerr('[push] %s server does not seem to respond', self.move_forward_server)
            return FTSMTransitions.INIT_FAILED
        return FTSMTransitions.INITIALISED

    def running(self):
        ## TODO: fill this method with the execution logic
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        grasp_successful = False
        if self.base_elbow_offset > 0:
            self.__align_base_with_pose(pose_base_link)

            # the base is now correctly aligned with the pose, so we set the
            # y position of the goal pose to the elbow offset
            pose_base_link.pose.position.y = self.base_elbow_offset

        if self.grasping_orientation:
            pose_base_link.pose.orientation.x = self.grasping_orientation[0]
            pose_base_link.pose.orientation.y = self.grasping_orientation[1]
            pose_base_link.pose.orientation.z = self.grasping_orientation[2]
            pose_base_link.pose.orientation.w = self.grasping_orientation[3]
        #opening the gripper
        rospy.loginfo('[push] Opening the gripper...')
        self.gripper.open()

        rospy.loginfo('[push] Preparing for grasp verification')
        self.gripper.init_grasp_verification()

        #move to the object
        rospy.loginfo('[push] Preparing sideways graps')
        pose_base_link = self.__prepare_sideways_grasp(pose_base_link)

        rospy.loginfo('[push] Grasping...')
        arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        
        if not arm_motion_success:
            rospy.logerr('[push] Arm motion unsuccessful')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE
        
        rospy.loginfo('[push] Arm motion successful')

        #grasping
        rospy.loginfo('[push] Closing the gripper')
        self.gripper.close()
        
        rospy.loginfo('[push] Verifying the grasp...')
        grasp_successful = self.gripper.verify_grasp()
        if grasp_successful:
            rospy.loginfo('[push] Successfully grasped object')
        else:
            rospy.loginfo('[push] Grasp unsuccessful')
                
        #move to goal position
        rospy.loginfo('[push] Moving the arm forward')
        #change self.safe_arm_joint_config to goal position
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.goal_config)
        
        #release the object
        rospy.loginfo('[push] Opening the gripper')
        self.gripper.open()

        # move to safe arm position
        rospy.loginfo('[push] Move the arm to safe position')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)


            
        
        return FTSMTransitions.DONE

    def recovering(self):
        ## TODO: if recovery behaviours are appropriate, fill this method with
        ## the recovery logic
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING
       
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

    def __prepare_sideways_grasp(self, pose_base_link):
        rospy.loginfo('[PICKUP] Moving to a pregrasp configuration...')
        if pose_base_link.pose.position.z > self.pregrasp_height_threshold:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)
        else:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_low_config_name)

        if self.intermediate_grasp_offset > 0:
            rospy.loginfo('[PICKUP] Moving to intermediate grasping pose...')
            pose_base_link.pose.position.x -= self.intermediate_grasp_offset
            self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        if self.intermediate_grasp_offset > 0:
            pose_base_link.pose.position.x += self.intermediate_grasp_offset
        return pose_base_link

    def set_result(self, success):
        result = PushResult()
        result.success = success
        return result
