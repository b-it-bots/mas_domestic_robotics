import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import moveit_commander
from geometry_msgs.msg import PoseStamped, WrenchStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
# from hsrb_interface import Robot


class CompositeBehaviours(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'composite_behaviours',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'composite_behaviours')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.retry_count = 0
        self.timeout = 120.
        #self.reference_frame = "odom"
        # robot= Robot()
        # self.wb_interface = robot.try_get('whole_body')

        self.joint_states_sub = rospy.Subscriber('/hsrb/joint_states', JointState, self.joint_states_cb)
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
        self.whole_body.set_planning_time(5)
        self.whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])
        #self.whole_body.set_pose_reference_frame(self.reference_frame)
        
        self.force_z = 0
        self.force_y = 0
        self.force_x = 0
        self.force_sub = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, self.save_force) 
        

    def execute(self, userdata):
        rospy.loginfo('[composite_behaviours] sample code for behaviours')

        # self.say("raise hand")
        # self.arm_to_named_target("raise_hand")
        # rospy.sleep(1)
        # self.say("going to neutral")
        # self.arm_to_named_target("neutral")
        # rospy.sleep(1)
        # self.say("gripper open")
        # self.control_gripper(0.0)
        # rospy.sleep(1)
        # self.say("head turn left")
        # self.control_head("head_pan_joint",1.57)
        # rospy.sleep(1)
        # self.say("head turn right")
        # self.control_head("head_pan_joint",0)
        # self.control_head("head_pan_joint",-1.57)
        # rospy.sleep(1)
        # self.control_arm_joint("wrist_roll_joint",-1.6)
        # rospy.sleep(1)
        # self.control_arm_joint("wrist_flex_joint",-1.0)
        # rospy.sleep(1)
        # self.wb_interface.move_to_joint_positions({'wrist_roll_joint': -1.5})
        # rospy.sleep(1)
        rospy.loginfo("gripper")

        #self.move_wholebody_ik(0.4,-0.05,0.1,180,0,0)

        # self.arm_to_named_target("pregrasp_top")
        # rospy.sleep(1)
        # self.control_arm_lift(0.5)   
        self.move_arm_down_and_check_force()                 
                
        return 'succeeded'
    

    def control_arm_lift(self,val):
        self.arm.set_joint_value_target("arm_lift_joint",val)       
        self.arm.go()
    
    def control_arm_joint(self,joint_name,val):
        self.arm.set_joint_value_target(joint_name,val)       
        self.arm.go()   
    
    def arm_to_named_target(self,named_config):
        self.arm.set_named_target(named_config)       
        self.arm.go()
        
    def control_gripper(self, val):
        self.gripper.set_joint_value_target("hand_motor_joint", val)
        self.gripper.go()
    
    def control_head(self, joint_name,joint_value):
        self.head.set_joint_value_target(joint_name, joint_value)
        self.head.go()
    
    def save_force(self,msg):
        self.force_x = msg.wrench.force.x
        self.force_y = msg.wrench.force.y
        self.force_z = msg.wrench.force.z

    def joint_states_cb(self, msg):
        self.joint_states = msg
    
    def move_wholebody_ik(self,x, y, z, roll, pitch, yaw):
        rospy.loginfo(x)
        rospy.loginfo(y)
        rospy.loginfo(z)
        rospy.loginfo(roll)
        rospy.loginfo(pitch)
        rospy.loginfo(yaw)

        # p = PoseStamped()        
        # p.header.frame_id = "hand_palm_link"        
        # p.pose.position.x = x
        # p.pose.position.y = y
        # p.pose.position.z = z        
        # odom_quat= quaternion_from_euler(roll, pitch, yaw)  
        # p.pose.orientation.x = odom_quat[0]
        # p.pose.orientation.y = odom_quat[1]
        # p.pose.orientation.z = odom_quat[2]
        # p.pose.orientation.w = odom_quat[3]

        p = PoseStamped()
        p.header.frame_id = "hand_palm_link"
        p.pose.position.z = 0.4
        p.pose.orientation.w = 1

        self.whole_body.set_pose_target(p)
        return self.whole_body.go()
    
    def move_arm_down_and_check_force(self, max_lift=0.69, min_lift=0, decrement_step=0.01, force_threshold=4):
        # Move the arm to the pregrasp lift position first
        rospy.loginfo("Going to pregrasp top")
        self.arm_to_named_target("pregrasp_top")
        rospy.sleep(1)
        self.control_gripper(-0.7) #Set value to close the gripper
        rospy.sleep(1)
        arm_lift_indx = self.joint_states.name.index('arm_lift_joint')
        
        # Continuously decrement the lift joint value
        while self.joint_states.position[arm_lift_indx] > min_lift:
            #Check the force value along the z-axis
            rospy.loginfo("z")
            rospy.loginfo(self.force_z)
            if self.force_z > force_threshold:
                # If the force is below the threshold, move the arm 0.05 cm up
                current_lift = self.joint_states.position[arm_lift_indx]
                new_lift = max(current_lift + 0.05, min_lift)
                self.control_arm_lift(new_lift)
                break  # Stop decrementing

            current_lift = self.joint_states.position[arm_lift_indx]
            new_lift = max(current_lift - decrement_step, min_lift)
            self.control_arm_lift(new_lift)
            rospy.sleep(0.1) 

        


