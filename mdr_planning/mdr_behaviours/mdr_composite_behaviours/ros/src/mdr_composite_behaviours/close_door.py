import rospy
import math
#from utils import *
import time
import actionlib
import random
import numpy as np
from std_msgs.msg import Bool,String,Float32
import sys
from mdr_composite_behaviours.publish_force import ForceSensorCapture
from mdr_composite_behaviours.force_feedback import ForceToVelocityNode

import control_msgs.msg
import trajectory_msgs.msg
import controller_manager_msgs.srv
from geometry_msgs.msg import PoseStamped,Twist 
from std_msgs.msg import Bool,Float32
from scipy import signal
import statistics
from geometry_msgs.msg import WrenchStamped

from mdr_pickup_action.msg import PickupAction, PickupGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mas_hsr_gripper_controller.gripper_controller import GripperController
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import rospy
from sensor_msgs.msg import JointState
import threading
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from mdr_composite_behaviours.composite_behaviours import CompositeBehaviours
# from mdr_composite_behaviours.base_manuver import move_in_forward_right_arc, move_in_forward_left_arc

#move_in_forward_right_arc(0.1, 0.9, 50) = opening the door
#move_in_forward_left_arc(0.1, 0.9, 50) = closing the door


#rospy.init_node('base_manuver')


import math

from geometry_msgs.msg import Twist

# 速度指令のパブリッシャーを作成
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

def move_base_vel(vx, vy, vw):
    u"""台車を速度制御する関数

    引数:
        vx (float): 直進方向の速度指令値 [m/s]（前進が正、後進が負）
        vy (float): 横方向の速度指令値 [m/s]（左が正、右が負）
        vw (float): 回転方向の速度指令値 [deg/s]（左回転が正、右回転が負）

    """

    # 速度指令値をセットします
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = -1*vw / 180.0 * math.pi  # 「度」から「ラジアン」に変換します
    base_vel_pub.publish(twist) 

# import math

def move_in_backward_right_arc(v, r, theta_degrees):
    # Calculate angular velocity in degrees per second
    omega = -(abs(v) * 180) / (r * math.pi)

    # Calculate time to complete the arc
    t = theta_degrees / abs(omega)

    # Get current time
    start_time = rospy.Time.now().to_sec()

    # Command the robot to move in the arc
    while rospy.Time.now().to_sec() - start_time < t:
        move_base_vel(v, 0, omega)  # Note: v should be passed as a negative value for backward motion
    
    # Stop the robot after completing the arc
    move_base_vel(0, 0, 0)

    #Call the function with a velocity of -0.5 m/s (backward), radius of 0.8 m, and an angle of 80 degrees
    #move_in_backward_right_arc(-0.1, 0.3, 80)

def move_in_forward_left_arc(v, r, theta_degrees):
    # Calculate angular velocity in degrees per second
    omega = -(abs(v) * 180) / (r * math.pi)

    # Calculate time to complete the arc
    t = theta_degrees / abs(omega)

    # Get current time
    start_time = rospy.Time.now().to_sec()

    # Command the robot to move in the arc
    while rospy.Time.now().to_sec() - start_time < t:
        move_base_vel(v, 0, omega)  # Note: v should be passed as a negative value for backward motion
    
    # Stop the robot after completing the arc
    move_base_vel(0, 0, 0)

def move_in_forward_left_arc_force(v, r, theta_degrees, force_thres):
    # Calculate angular velocity in degrees per second
    omega = -(abs(v) * 180) / (r * math.pi)

    # Calculate time to complete the arc
    t = theta_degrees / abs(omega)

    # Get current time
    start_time = rospy.Time.now().to_sec()

    # Command the robot to move in the arc
    while rospy.Time.now().to_sec() - start_time < t and force_thres >= 70:
        move_base_vel(v, 0, omega)  # Note: v should be passed as a negative value for backward motion
    
    # Stop the robot after completing the arc
    move_base_vel(0, 0, 0)

def move_in_forward_right_arc(v, r, theta_degrees):
    # Calculate angular velocity in degrees per second
    omega = -(abs(v) * 180) / (r * math.pi)

    # Calculate time to complete the arc
    t = theta_degrees / abs(omega)

    # Get current time
    start_time = rospy.Time.now().to_sec()

    # Command the robot to move in the arc
    while rospy.Time.now().to_sec() - start_time < t:
        move_base_vel(v, 0, -1*omega)  # Note: v should be passed as a negative value for backward motion
    
    # Stop the robot after completing the arc
    move_base_vel(0, 0, 0)



def compute_difference(pre_data_list, post_data_list,initial,post):
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
   
    
    angle=np.degrees(initial)
    
    # Applying transformation based on rotation about Z axis
    x_new=post_data_list[0]*math.cos(angle)-post_data_list[1]*math.sin(angle)
    y_new=post_data_list[0]*math.sin(angle)+post_data_list[1]*math.cos(angle)
    z_new=post_data_list[2]

    x_old=pre_data_list[0]
    y_old=pre_data_list[1]
    z_old=pre_data_list[2]

    # Calculate square sum of difference
    result=math.sqrt(math.pow(x_old-x_new,2)+math.pow(y_old-y_new,2)+math.pow(z_old - z_new,2))
    return result

def get_max_directional_force(x, y, z):
    magnitudes = [abs(x), abs(y), abs(z)]
    max_magnitude = max(magnitudes)
    
    if max_magnitude == abs(x):
        force = x
    elif max_magnitude == abs(y):
        force = y
    else:
        force = z

    direction = math.degrees(math.atan2(z, y))
    

    return direction

class CloseDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'close_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['lever_pose'],
                                   output_keys=['wrist_direction'])
        
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'close_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.lever_pose = list(kwargs.get('lever_pose', list()))
        self.retry_count = 0
        self.timeout = 120.
        self.forceCapture=None
        self.node=None

        self.door_direction=None
               
        # intialize gripper controllerr
        self.gripper_controller = GripperController()
        self.action_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.action_cli.wait_for_server()
        # rospy.loginfo("Connected to server for executing door opening")
        self.speak=1
        self.wrist_direction = None
        self.say_pub = rospy.Publisher('/say', String, latch=True, queue_size=1)
        self.pub  = rospy.Publisher('Handle_unlatched', Bool, queue_size=10)
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.traj = trajectory_msgs.msg.JointTrajectory()
        self.traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.p = trajectory_msgs.msg.JointTrajectoryPoint()
        #receive torques
        self.torque_sub = rospy.Subscriber('/hsrb/wrist_wrench/compensated', Bool, self.save_torque) ## subscriber in plot juggler (for both force and torque threshold feedback)
        self.torque_val = 0
        self.force_feedback_sub = rospy.Subscriber('force_threshold', Bool, self.get_force_feedback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        #initialising the client for moving arm to neutral position
        #initialising the client for moving arm to neutral position
        # try:
        #     self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
        #     rospy.loginfo('[door_open] Waiting for %s server', "move_arm_server")
        #     self.move_arm_client.wait_for_server()
        # except Exception as exc:
        #     rospy.logerr('[door_open] %s server does not seem to respond: %s',
        #                 "move_arm_server", str(exc))
        # print("All good!!")
        ##===========================================================================
        
        ## initialization for moveit arm

                #self.reference_frame = "odom"
        self.arm = moveit_commander.MoveGroupCommander("arm",
                                                  wait_for_servers=0.0)
        self.base = moveit_commander.MoveGroupCommander("base",
                                                   wait_for_servers=0.0)
        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=0.0)
        self.head = moveit_commander.MoveGroupCommander("head",
                                                   wait_for_servers=0.0)
        self.whole_body \
            = moveit_commander.MoveGroupCommander("whole_body_light",
                                                  wait_for_servers=0.0)
        self.whole_body.allow_replanning(True)
        self.lever_pose = list(kwargs.get('lever_pose', dict()))
        self.whole_body.set_planning_time(5)
        self.whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])
        self.arm.set_pose_reference_frame('base_link')

    def get_force_feedback(self, msg):
        if msg.data and self.speak:
            rospy.loginfo('[door_open]Cannot pull. Force feedback exceeds threshold. Trying to push...')
            self.speak=0
            self.say("Cannot pull. Trying to push.")

    def save_torque(self,msg):
        self.torque_val=msg.wrench.torque.x

    def get_door_handle_allignment(self):
        #decide clockwise or anticlockwise rotation
        self.p.positions= [0.35, -0.42, 0.0, -1.00, np.round(np.deg2rad(-55), 2)]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Duration(1)
        self.traj.points = [self.p]
        self.goal.trajectory = self.traj
        self.action_cli.send_goal(self.goal)
        self.action_cli.wait_for_result()
        time.sleep(1)
        if self.torque_val>0.5:   
            self.p.positions= [0.35, -0.42, 0.0, -1.00, np.round(np.deg2rad(-135), 2)]
            self.p.time_from_start = rospy.Duration(1)
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.action_cli.send_goal(self.goal)
            print(self.action_cli.wait_for_result())
            #Anti-clockwise wrist rotation
            self.wrist_direction='acw'
        else:
            #Clockwise wrist rotation
            self.wrist_direction='cw'
        self.say(str(self.wrist_direction))
    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)

    
    def moveToNeutral(self):
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        move_arm_goal.named_target = "neutral"
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        self.move_arm_client.get_result()
        rospy.loginfo("Back to neutral position")
        rospy.sleep(5)

    def control_gripper(self, val):
        self.gripper.set_joint_value_target("hand_motor_joint", val)
        self.gripper.go()    
    

    def one_func(self):
        self.speak=1
        # open gripper by default
        self.say("Through the door")
        self.gripper_controller.open()
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        # Move to initial grabbing position
        angles= list(range(0, -100, -15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles:
            p.positions= [0.35, -0.42, 0.0, -1.00, i]
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result()
        # close gripper arm

       ## wrist_roll_angle=-pi/2

        # return 'succeeded'

        rospy.loginfo("=======================================")
        rospy.loginfo("arm pose function finished")

        self.gripper_controller.close()
        rospy.loginfo('Door Handle Grasped')
        handle = self.get_door_handle_allignment()
        ## second stage
        if self.wrist_direction == 'acw':
            angles= list(range(-135, -145, -15))
        elif self.wrist_direction == 'cw':
            angles= list(range(-55, -45, 15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles:
            p.positions= [0.20, -0.42, 0.0, -1.00, i]
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result()
        # close gripper arm
        self.gripper_controller.close()
        rospy.loginfo('Door Handle Unlatched')
        rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     rospy.sleep(0.1)
        
        rospy.sleep(5)
        self.control_gripper(0.0)
        move_in_forward_right_arc(0.1, 0.9, 50) # opening the door
    #  ##---------------- zain commented------------------------  
    #     rospy.loginfo('Received force feedback')
    #     cmd_vel_msg = Twist()
    #     # cmd_vel_msg.linear.x = -0.05
    #     cmd_vel_msg.linear.x = 0.05
    #     self.pub_cmd_vel.publish(cmd_vel_msg)
    #     time.sleep(0.5)
    #     cmd_vel_msg.linear.x = 0.0
    #     self.pub_cmd_vel.publish(cmd_vel_msg)
    # ##---------------- zain commented------------------------
        # p.positions= [0.35, -0.42, 0.0, -1.00, np.round(np.deg2rad(-90), 2)]
        # p.velocities = [0, 0, 0, 0, 0]
        # p.time_from_start = rospy.Duration(1)
        # traj.points = [p]
        # goal.trajectory = traj
        # self.action_cli.send_goal(goal)
        # self.action_cli.wait_for_result()
        # now move back a bit
        #self.movebackwards()
        #rospy.loginfo('Moved back a bit')
  
       
    def arm_to_named_target(self,named_config):
        self.arm.set_named_target(named_config)       
        self.arm.go()
        
    def control_gripper(self, val):
        self.gripper.set_joint_value_target("hand_motor_joint", val)
        self.gripper.go()


    def execute(self, userdata):
        rospy.loginfo('[close_door] Trying to close the door')
        # rospy.loginfo('[close_door] i am in whole run file')
        self.say('In state close door')
        # self.say('Im using whole file')
        self.say('Trying to close the door') 
        #self.moveToNeutral()
        # self.say("raise hand")
        # self.arm_to_named_target("raise_hand")
        # rospy.sleep(1)
        # self.say("going to neutral")
        # self.arm_to_named_target("neutral")
        # rospy.sleep(1)
        # self.say("gripper open")
        # self.control_gripper(0.0)
        self.arm.set_named_target("erl_handle_pregrasp")       
        self.arm.go()
        rospy.sleep(1)
        self.say("gripper close")
        self.control_gripper(-0.7)
        rospy.sleep(1)
        move_in_forward_left_arc(0.1, 0.9, 50)
        # pick_pour= pickAndPour()
        # self.lever_pose=userdata.lever_pose
        # rospy.loginfo("User data lever pose: ")
        # rospy.loginfo(userdata.lever_pose)
        # self.one_func()
        # userdata.wrist_direction =self.wrist_direction
        
        rospy.loginfo('[close_door] one func finished')


        # self.forceCapture=ForceSensorCapture()
        # force_capture_thread = threading.Thread(target=self.forceCapture.calculate_force)
        # force_capture_thread.start()
        # rospy.loginfo('[open_door] ForceSensorCapture instantiated')
        # self.forceCapture.calculate_force()
        # rospy.loginfo("Done until force calculated")
        # self.node = ForceToVelocityNode()
        # # rospy.loginfo("[open_door]ForceToVelocityNode object instantiated")
        # # self.node.run()
        # # rospy.loginfo("[open_door] run complete")
         
        #     # Create a thread for forceCapture.calculate_force()
      
        

        # # Create another thread for node.run()
        # node_run_thread = threading.Thread(target=self.node.run)
        # node_run_thread.start()


        # Optionally, you can wait for those threads to finish if needed
        # force_capture_thread.join()
        # node_run_thread.join()
              
        return 'succeeded'

# def main():
#     door_open= OpenDoor()
#     door_open.one_func()
#     rospy.spin()
# if __name__== "__main__" :
#     main()
