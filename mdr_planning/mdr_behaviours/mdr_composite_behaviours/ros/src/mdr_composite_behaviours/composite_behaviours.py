import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import moveit_commander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


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

    def execute(self, userdata):
        rospy.loginfo('[composite_behaviours] sample code for behaviours')

        # self.say("raise hand")
        # self.arm_to_named_target("raise_hand")
        # rospy.sleep(1)
        # self.say("going to neutral")
        # self.arm_to_named_target("neutral")
        # rospy.sleep(1)
        # self.say("gripper open")
        # self.control_gripper(0.2)
        # rospy.sleep(1)
        # self.say("head turn left")
        # self.control_head("head_pan_joint",1.57)
        # rospy.sleep(1)
        # self.say("head turn right")
        # self.control_head("head_pan_joint",0)
        # self.control_head("head_pan_joint",-1.57)
        # rospy.sleep(1)

        self.say("Whole body move")

        rospy.loginfo(self.whole_body.get_end_effector_link())

        self.move_wholebody_ik(0.4,-0.05,0.1,180,0,0)

        
        return 'succeeded'
    

    def arm_to_named_target(self,named_config):
        self.arm.set_named_target(named_config)       
        self.arm.go()
        
    def control_gripper(self, val):
        self.gripper.set_joint_value_target("hand_motor_joint", val)
        self.gripper.go()
    
    def control_head(self, joint_name,joint_value):
        self.head.set_joint_value_target(joint_name, joint_value)
        self.head.go()
    
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
