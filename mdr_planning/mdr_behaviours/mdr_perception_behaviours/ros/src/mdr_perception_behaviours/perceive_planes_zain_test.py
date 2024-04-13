import time
import rospy
import moveit_commander
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from std_msgs.msg import String

from mas_execution_manager.scenario_state_base import ScenarioStateBase


class PerceivePlanes(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'perceive_plane',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying',],
                                   input_keys=['object_tilt'],
                                   output_keys= ['shelf','all_arm_joints','all_head_joints'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'perceive_planes')
        self.timeout = kwargs.get('timeout', 120.)
        self.plane_prefix = kwargs.get('plane_prefix', 0)
        # self.tilt_angle = kwargs.get('object_tilt', -0.1)
        # self.tilt_angle=kwargs.get('object_tilt', -0.2)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.clear_memory= kwargs.get('clear_memory', False)
        self.retry_count = 0
        rospy.Subscriber('/germanopen/store_groceries/object_category', String, self.set_object)
        self.cat = None
        rospy.logerr('Initialising moveit group')
        self.head = moveit_commander.MoveGroupCommander("head")
        self.arm = moveit_commander.MoveGroupCommander("arm")
        rospy.logerr('moveit group initialized')
    
    
    def set_object(self, data):
        self.cat = data
        rospy.logdebug(f'shelf {self.cat}')
        
           

    def execute(self, userdata):
    #     if userdata.object_tilt:
    #         self.tilt_angle = userdata.object_tilt
    #     else:
    #         userdata.object_tilt=self.tilt_angle
        rospy.logwarn(f"============== Object Catagory received:{self.cat} ==================")
        if self.cat=="1":
            # self.tilt_angle = -0.4
             self.arm_joints= {'arm_flex_joint': 0.0, 'arm_roll_joint': 1.57, 'arm_lift_joint': 0.0, 'wrist_roll_joint': 0.0, 'wrist_flex_joint': -1.57}
             self.head_joints={'head_pan_joint': 0.0, 'head_tilt_joint': -0.4}
            
        else:
            self.arm_joints= {'arm_flex_joint': 0.0, 'arm_roll_joint': 1.57, 'arm_lift_joint': 0.5, 'wrist_roll_joint': 0.0, 'wrist_flex_joint': -1.57}
            self.head_joints={'head_pan_joint': 0.0, 'head_tilt_joint': -0.2}
            # self.tilt_angle = -0.2

        userdata.all_head_joints=self.head_joints
        userdata.all_arm_joints=self.arm_joints
        # self.head.set_joint_value_target("head_tilt_joint", self.tilt_angle)
        # self.head.go()

        

        if self.save_sm_state:
            self.save_current_state()
        clear_memory = self.clear_memory  # Assuming you have a method to decide this
        dispatch_msg = self.get_dispatch_msg(self.plane_prefix, clear_memory=clear_memory)
        # dispatch_msg = self.get_dispatch_msg(self.plane_prefix)
        rospy.loginfo('Perceiving %s' % self.plane_prefix)
        self.say('Perceiving ' + self.plane_prefix)
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            self.say('%s perceived' % self.plane_prefix)
            rospy.loginfo('%s perceived successfully' % self.plane_prefix)
            return 'succeeded'

        rospy.loginfo('Could not perceive %s' % self.plane_prefix)
        self.say('Could not perceive ' + self.plane_prefix)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to perceive %s' % self.plane_prefix)
            self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to perceive %s' % self.plane_prefix)
        self.retry_count += 1
        return 'failed'

    # def get_dispatch_msg(self, plane_name):
    #     dispatch_msg = plan_dispatch_msgs.ActionDispatch()
    #     dispatch_msg.name = self.action_name

    #     arg_msg = diag_msgs.KeyValue()
    #     arg_msg.key = 'bot'
    #     arg_msg.value = self.robot_name
    #     dispatch_msg.parameters.append(arg_msg)

    #     arg_msg = diag_msgs.KeyValue()
    #     arg_msg.key = 'plane'
    #     arg_msg.value = plane_name
    #     dispatch_msg.parameters.append(arg_msg)

    #     return dispatch_msg
    def get_dispatch_msg(self, plane_name, clear_memory=False):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        # Bot parameter
        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        # Plane parameter
        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'plane'
        arg_msg.value = plane_name
        dispatch_msg.parameters.append(arg_msg)

        # Clear memory parameter
        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'clear_memory'
        arg_msg.value = str(clear_memory).lower()  # Send 'true' or 'false' as a string
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg
