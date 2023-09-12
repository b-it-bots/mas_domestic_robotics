import rospy
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

class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['lever_pose']))
        
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.lever_pose = list(kwargs.get('lever_pose', list()))
        self.retry_count = 0
        self.timeout = 120.
        self.forceCapture=None
        self.node=None
               
                # intialize gripper controllerr
        self.gripper_controller = GripperController()

        #initialising the action client for picking
        self.client_pick = actionlib.SimpleActionClient('pickup_server', PickupAction)
        rospy.loginfo('waiting for server')
        self.client_pick.wait_for_server()
        rospy.loginfo('Done waiting for server')

       ### this part can be removed === by zain
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
        # self.sub_force = rospy.Subscriber('/max_force', Float32, self.force_callback)
        # self.pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

        ##===========================================================================
        


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
        

    def get_force_feedback(self, msg):
        if msg.data and self.speak:
            self.direction_multiplier = 1
            rospy.logerr('[door-open]Cannot pull. Force feedback exceeds threshold. Trying other direction...')
            self.speak=0
            self.say("Cannot pull. Force feedback exceeds threshold.")
    
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
        # addition for the hand motion based on coordinates
        # if len(self.lever_pose) == 0:
        #     self.lever_pose = userdata.lever_pose
        #     rospy.loginfo("Using userdata's lever_pose {0}".format(self.lever_pose))
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


            p.time_from_start = rospy.Duration(1)
            traj.points = [p]
            goal.trajectory = traj
            self.action_cli.send_goal(goal)
            self.action_cli.wait_for_result()
  ### until the arm reaches the lever
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

        # now move back a bit
        #self.movebackwards()
        #rospy.loginfo('Moved back a bit')

    # def calculate_force(self):

    #     pub = rospy.Publisher('force',Bool, queue_size = 10)
    #     pub_force = rospy.Publisher('force_values',Float32, queue_size = 10)
    #     pub_max_force = rospy.Publisher('max_force',Float32, queue_size = 10)
    #     # Start force sensor capture
    #     force_sensor_capture = ForceSensorCapture()

    #     # Get initial data of force sensor
    #     pre_force_list = force_sensor_capture.get_current_force()
        
    #     pre_angle=force_sensor_capture.get_current_angle()


    #     # Wait until force sensor data become stable
    #     rospy.sleep(1.0)
        
    #     # Getting current force
    #     post_force_list = force_sensor_capture.get_current_force()
    #     post_angle=force_sensor_capture.get_current_angle()



    #     force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
    

    #     rospy.Rate(10)
    #     median_angles_list=[]
    #     while not rospy.is_shutdown():
    #         # Getting new force sensor reading
    #         for i in range(5):
    #             post_force_list = force_sensor_capture.get_current_force()
            
    #         # Getting current angle
    #             post_angle=force_sensor_capture.get_current_angle()

    #         #print(post_force_list[0][0],post_force_list[1][0],post_force_list[2][0])
    #             max_force = get_max_directional_force(post_force_list[0][0],post_force_list[1][0],post_force_list[2][0])
    #             median_angles_list.append(max_force)
    #         med = statistics.median(median_angles_list)
    #         print(med)
    #         pub_max_force.publish(med)
    #         median_angles_list=[]


    #         # Computing difference from initial and new force sensor readings

    #         force_difference = compute_difference(pre_force_list, post_force_list,pre_angle,post_angle)
    #         pub_force.publish(force_difference)
    #         if force_difference > 45:
    #             pub.publish(True)
    #         else:
    #             pub.publish(False)
    #         #print(force_difference)
    #         rospy.sleep(0.1)
    
    def create_force_direction_node(self):
            node = ForceToVelocityNode()
            node.run()
       



    def execute(self, userdata):
        rospy.loginfo('[open_door] Trying to open the door')
        rospy.loginfo('[open_door] i m in whole run file')
        self.say('In state open door')
        self.say('Im using whole file')
        self.say('Trying to open the door') 
        # pick_pour= pickAndPour()
        self.one_func()
        rospy.loginfo('[open_door] one func finished')


        self.forceCapture=ForceSensorCapture()
        force_capture_thread = threading.Thread(target=self.forceCapture.calculate_force)
        force_capture_thread.start()
        # rospy.loginfo('[open_door] ForceSensorCapture instantiated')
        # self.forceCapture.calculate_force()
        # rospy.loginfo("Done until force calculated")
        self.node = ForceToVelocityNode()
        # rospy.loginfo("[open_door]ForceToVelocityNode object instantiated")
        # self.node.run()
        # rospy.loginfo("[open_door] run complete")
         
            # Create a thread for forceCapture.calculate_force()
      
        

        # Create another thread for node.run()
        node_run_thread = threading.Thread(target=self.node.run)
        node_run_thread.start()


        # Optionally, you can wait for those threads to finish if needed
        # force_capture_thread.join()
        # node_run_thread.join()
              
        return 'succeeded'