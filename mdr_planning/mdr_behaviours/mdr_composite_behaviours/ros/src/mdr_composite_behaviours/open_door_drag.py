import rospy
import time
import actionlib
import random
import numpy as np
from std_msgs.msg import Bool,String,Float32
import sys

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
import moveit_commander
from sensor_msgs.msg import JointState

from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)


class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.retry_count = 0
        self.timeout = 120.
        self.linear_velocity = -0.05
        self.angular_velocity = 0.0
        self.linear_velocity_y = 0.0
        self.angular_published = False

        self.force_threshold = -30.0
        self.force_another_threshold = -50
        self.direction = None # earlier is_left
        self.in_loop = False
        self.trend_value_count = 0
        self.force_trend_values = []
        self.sub_force = rospy.Subscriber('/max_force', Float32, self.force_callback)
        self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.wrist_roll_sub=rospy.Subscriber('/hsrb/joint_states',JointState,self.__angle_cb)
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        self.wrist_roll_initial_pos=0.0
        self.wrist_roll_post_pos=0.0
        self.wrist_roll_angle=0.0

                # Here we are using compensated data
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        self.wrist_roll_sub=rospy.Subscriber('/hsrb/joint_states',JointState,self.__angle_cb)

        
        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
        
        rospy.loginfo(" ")
        rospy.loginfo("=" *20)
        rospy.loginfo("arm joint angles: ")
        rospy.loginfo(self.arm.get_current_joint_values())
        rospy.loginfo("=" *20)
        rospy.loginfo(" ")
                # intialize gripper controllerr
        self.gripper_controller = GripperController()

        #initialising the action client for picking
        self.client_pick = actionlib.SimpleActionClient('pickup_server', PickupAction)
        rospy.loginfo('waiting for server')
        self.client_pick.wait_for_server()
        rospy.loginfo('Done waiting for server')

          #initialising the action client for moving sideways
        try:
            self.move_base_client = actionlib.SimpleActionClient("move_base_server", MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_base_server")
            self.move_base_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         "move_base_server", str(exc))     

        # initialising the action client for pouring
        self.action_cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.action_cli.wait_for_server()
        rospy.loginfo("Connected to server for executing pouring")

        
        self.speak=1
        self.direction_multiplier = 1
        self.say_pub = rospy.Publisher('/say', String, latch=True, queue_size=1)
        self.pub  = rospy.Publisher('Handle_unlatched', Bool, queue_size=10)
        #initialising the client for moving arm to neutral position
        try:
            self.move_arm_client = actionlib.SimpleActionClient("move_arm_server", MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', "move_arm_server")
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                        "move_arm_server", str(exc))

    def __angle_cb(self,data):
        self.wrist_roll_angle=data.position[-1]


    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]
    # Added separately
    def get_current_angle(self):
        return self.wrist_roll_angle    

    def get_force_feedback(self, msg):
        if msg.data and self.speak:
            self.direction_multiplier = 1
            rospy.logerr('[door-open]Cannot pull. Force feedback exceeds threshold. Trying other direction...')
            self.speak=0
            self.say("Cannot pull. Force feedback exceeds threshold.")
    
    def __ft_sensor_cb(self, data):
        
        # Getting force as three components
        force_data_x = data.wrench.force.x
        force_data_y = data.wrench.force.y
        force_data_z = data.wrench.force.z
        
        # Applying low pass filter for force values

        FX = []
        FY = []
        FZ = []

        FX.append(force_data_x)
        FY.append(force_data_y)
        FZ.append(force_data_z)

        # Sampling frequency
        fs = 124.95 
        
        # Cut-off frequency
        fc = 55

        w = fc / (fs/2)

        # Using function for lowpass butterworth filter
        b,a = signal.butter(2,w,'low')
        
        self._force_data_x = signal.lfilter(b, a, FX) #Forward filter
        self._force_data_y = signal.lfilter(b, a, FY)
        self._force_data_z = signal.lfilter(b, a, FZ)

        # Reference for lowpass filter: https://answers.ros.org/question/312833/how-do-i-implement-a-low-pass-filter-to-reduce-the-noise-coming-from-a-topic-that-is-publishing-a-wrenchstamped-msg-type-using-a-python-script/


    def calculate_force():

        pub = rospy.Publisher('force',Bool, queue_size = 10)
        pub_force = rospy.Publisher('force_values',Float32, queue_size = 10)
        pub_max_force = rospy.Publisher('max_force',Float32, queue_size = 10)
        # Start force sensor capture
        force_sensor_capture = ForceSensorCapture()

        # Get initial data of force sensor
        pre_force_list = force_sensor_capture.get_current_force()
        
        pre_angle=force_sensor_capture.get_current_angle()


        # Wait until force sensor data become stable
        rospy.sleep(1.0)
        
        # Getting current force
        post_force_list = force_sensor_capture.get_current_force()
        post_angle=force_sensor_capture.get_current_angle()



        force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
    

        rospy.Rate(10)
        median_angles_list=[]
        while not rospy.is_shutdown():
            # Getting new force sensor reading
            for i in range(5):
                post_force_list = force_sensor_capture.get_current_force()
            
            # Getting current angle
                post_angle=force_sensor_capture.get_current_angle()

            #print(post_force_list[0][0],post_force_list[1][0],post_force_list[2][0])
                max_force = get_max_directional_force(post_force_list[0][0],post_force_list[1][0],post_force_list[2][0])
                median_angles_list.append(max_force)
            med = statistics.median(median_angles_list)
            print(med)
            pub_max_force.publish(med)
            median_angles_list=[]


            # Computing difference from initial and new force sensor readings

            force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
            pub_force.publish(force_difference)
            if force_difference > 45:
                pub.publish(True)
            else:
                pub.publish(False)
            #print(force_difference)
            rospy.sleep(0.1)


    
    def pick(self):
        #min_height = 0.82 
        #max_height = 0.95
        #pickup_height = random.uniform(min_height, max_height)

        goal = PickupGoal()
        #goal.context = PickupGoal.CONTEXT_MOVING
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = rospy.Time.now()

        # goal.pose.pose.position.x = 0.418
        # goal.pose.pose.position.y = 0.078
        # goal.pose.pose.position.z = 0.842
        # goal.pose.pose.position.z = 0.95
        
        # goal.pose.pose.orientation.x = 0.758
        # goal.pose.pose.orientation.y = 0.000
        # goal.pose.pose.orientation.z = 0.652
        # goal.pose.pose.orientation.w = 0.000

        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 0.0

        rospy.loginfo('sending goal')
        self.client_pick.send_goal(goal)
        self.client_pick.wait_for_result()
        rospy.loginfo('got result')
        
        rospy.loginfo(self.client_pick.get_result())
        rospy.sleep(5)

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
        
    def one_func(self):
        self.speak=1
        # open gripper by default
        self.say("Through the door")
        self.gripper_controller.open()
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()


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
        self.gripper_controller.close()
        rospy.loginfo('Door Handle Grasped')

        # unlatching process  
        # first stage - greesn door handle inside
        # first stage - greesn door handle inside
        angles= list(range(-100, -135, -15))
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
        
        ## second stage 
        # second stage - green door handle inside
        # second stage - green door handle inside
        angles= list(range(-135, -145, -15))
        inRadians= np.deg2rad(angles)
        wrist_roll_angles= np.round(inRadians, 2)
        for i in wrist_roll_angles: 
            p.positions= [0.31, -0.42, 0.0, -1.00, i]
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result() 
            self.action_cli.wait_for_result() 

        

        # close gripper arm
        self.gripper_controller.close()
        rospy.loginfo('Door Handle Unlatched')
        rospy.Rate(10)
        while not rospy.is_shutdown():
            self.force_feedback_sub = rospy.Subscriber('force', Bool, self.get_force_feedback)
            rospy.sleep(0.1)
        rospy.loginfo('Received force feedback')
        # now move back a bit
        #self.movebackwards()
        #rospy.loginfo('Moved back a bit')
    
    
    def check_force_trend(self, tolerance_threshold = 5, min_streak_length = 3):
        print(self.force_trend_values)
        dir = None
        streak = 0
        max_streak = 0
        is_trend_decreasing = False
        
        for i in range(1, len(self.force_trend_values)):
            if self.force_trend_values[i] <= self.force_trend_values[i - 1] + tolerance_threshold:
                streak += 1
                max_streak = max(max_streak, streak)
                if streak >= min_streak_length:
                    is_trend_decreasing = True
            else:
                streak = 0
        # print(max_streak)

        if is_trend_decreasing == True:
            dir = 'Left'
        else:
            dir = 'Right'
        
        print(dir)
        return dir
    def force_callback(self, force_msg):

        if force_msg.data > -30.0 and self.in_loop == False:
            # print("first loop")
            self.linear_velocity = -0.01
            self.linear_velocity_y = 0.0

        else:
            # print("we in else")
            print(self.direction)
            self.in_loop = True
            if force_msg.data > 15.0:
                self.in_loop = False
            if self.direction == None:
                self.linear_velocity_y = -0.01
                self.decide_direction_of_movement(force_msg.data)
            

            self.linear_velocity = 0.0

            if self.direction == 'Right':# and force_msg.data > -33.0:
                # self.in_loop = True
                # if force_msg.data > 15.0:
                #     self.in_loop = False
                print("going right")
                print(self.direction)
                self.linear_velocity_y = -0.01
            if self.direction == 'Left':
                self.linear_velocity_y = 0.0
                # print("In else")
                # if force_msg.data < -33.0: 
                # self.in_loop = True
                # if force_msg.data > 15.0:
                #     self.in_loop = False
                print(force_msg.data)
                print("going left")
                self.linear_velocity_y = 0.01
                self.linear_velocity = 0.0
    #    if force_msg.data < self.force_threshold:
    #        self.angular_published = True
    #        print("here")
    #        #self.angular_velocity = -0.1  # Set angular velocity only once
    #        self.linear_velocity = 0.0
    #        self.linear_velocity_y = -0.03
    #        if force_msg.data < (self.force_threshold - 15):
    #            self.linear_velocity_y = 0.03
    #    else:
    #        print("loop")
    #        self.angular_velocity = 0.0
    #        self.linear_velocity_y = 0.0
    #        self.linear_velocity = -0.05

       

        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = -0.1#self.linear_velocity
        # cmd_vel_msg.linear.y = 0.0#self.linear_velocity_y
        # cmd_vel_msg.angular.z = 0.0#self.angular_velocity

        # self.pub_cmd_vel.publish(cmd_vel_msg)

    def shutdown(self):
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.linear_velocity
            cmd_vel_msg.linear.y = self.linear_velocity_y
            cmd_vel_msg.angular.z = self.angular_velocity

            self.pub_cmd_vel.publish(cmd_vel_msg)

            rate.sleep()

    def decide_direction_of_movement(self, force_angle):
        if self.trend_value_count <= 50: # TODO: change this to checking length of list and clean list after done
            # self.linear_velocity_y = -0.01 # pass this back to callback
            self.force_trend_values.append(force_angle)
            self.trend_value_count += 1
            # print("decide direction", self.force_trend_values)
        else:
            self.direction  = self.check_force_trend()

        
        # if self.trend_value_set == True:
        #    self.direction  = check_force_trend(self.force_trend_values)


    def execute(self, userdata):
        rospy.loginfo('[open_door] Trying to open the door')
        self.say('In state open door')
        self.say('Trying to open the door') 
        # pick_pour= pickAndPour()
        self.one_func()
        self.calculate_force()
        self.run()
         
              
        return 'succeeded'