from importlib import import_module
import rospy
import actionlib
from mas_tools.ros_utils import get_package_path
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mas_hsr_move_arm_joints_action.msg import MoveArmJointsGoal, MoveArmJointsAction


class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state, input_keys=['handle_pose'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 15)

        # get gripper controller
        gripper_package = kwargs.get('gripper_package', 'mdr_gripper_controller')
        gripper_module = '{0}.gripper_controller'.format(gripper_package)
        rospy.loginfo("importing 'GripperController' from '{}' module".format(gripper_module))
        # GripperControllerClass = getattr(import_module(gripper_module), 'GripperController')
        # self.gripper_controller = GripperController()

        # move arm action
        dmp_weight_paths = kwargs.get('dmp_weight_paths', list())
        self.dmp_name = get_package_path(*dmp_weight_paths)
        self.dmp_tau = kwargs.get('dmp_tau', 1)
        rospy.loginfo("using dmp_tau={} dmp_name='{}'".format(self.dmp_tau, self.dmp_name))
        move_arm_server_name = kwargs.get('move_arm_server', '/move_arm_server')
        self.move_arm_client = actionlib.SimpleActionClient(move_arm_server_name, MoveArmAction)
        rospy.loginfo('[push_door] Waiting for %s server', move_arm_server_name)
        if not self.move_arm_client.wait_for_server(rospy.Duration.from_sec(self.timeout)):
            raise RuntimeError("[open_door] timeout waiting for '{}' server".format(move_arm_server_name))

        # move arm joints action
        move_arm_joints_server_name = kwargs.get('action_server', '/mas_hsr_move_arm_joints_server')
        self.move_arm_joints_client = actionlib.SimpleActionClient(move_arm_joints_server_name, MoveArmJointsAction)
        if not self.move_arm_client.wait_for_server(rospy.Duration.from_sec(self.timeout)):
            raise RuntimeError("[open_door] failed to wait for '{}' server".format(move_arm_joints_server_name))

    def execute(self, userdata):
        self.say('trying to open door')
        rospy.loginfo('handle pose: {} {} {}'.format(userdata.handle_pose.pose.position.x,
                                                     userdata.handle_pose.pose.position.y,
                                                     userdata.handle_pose.pose.position.z))
        # move arm
        arm_goal = MoveArmGoal()
        # to neutral
        arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        arm_goal.named_target = 'neutral'
        self.move_arm_client.send_goal(arm_goal)
        self.move_arm_client.wait_for_result(rospy.Duration.from_sec(self.timeout))

        # to pose
        arm_goal.goal_type = MoveArmGoal.END_EFFECTOR_POSE
        arm_goal.dmp_name = self.dmp_name
        arm_goal.dmp_tau = self.dmp_tau
        arm_goal.end_effector_pose = userdata.handle_pose
        # self.move_arm_client.send_goal(arm_goal)
        # self.move_arm_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        # if self.number_of_retries > 0:
        #     self.number_of_retries -= 1
        #     return 'failed'
        # return 'failed_after_retrying'

        joint_goal = MoveArmJointsGoal()
        joint_goal.arm_joint_names = ['arm_flex_joint', 'arm_roll_joint',
                                      'arm_lift_joint', 'wrist_roll_joint', 'wrist_flex_joint']
        joint_goal.arm_joint_values = [0.0, 0.0, 0.23, -1.57, 0.]
        self.move_arm_joints_client.send_goal(joint_goal)
        self.move_arm_joints_client.wait_for_result(rospy.Duration.from_sec(self.timeout))

        # rotate gripper
        # self.gripper_controller.rotate_wrist(1.57)
        return 'failed_after_retrying'


class AskToOpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'ask_to_open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'ask_to_open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 15)

    def execute(self, userdata):
        self.say('I cannot open the door myself, can you please open the door')
        rospy.sleep(5.0)
        return 'succeeded'


class PushDoorOpen(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'push_door_open',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'push_door_open')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 15)

        # move forward
        self.movement_duration = kwargs.get('movement_duration', 5.)
        self.speed = kwargs.get('movement_speed', 0.1)
        move_forward_server_name = kwargs.get('move_forward_server', '/move_forward_server')
        self.move_forward_client = actionlib.SimpleActionClient(move_forward_server_name, MoveForwardAction)
        rospy.loginfo('[push_door] Waiting for %s server', move_forward_server_name)
        if not self.move_forward_client.wait_for_server(rospy.Duration.from_sec(self.timeout)):
            raise RuntimeError("[push_door] timeout waiting for '{}' server".format(move_forward_server_name))

        # move arm action
        move_arm_server_name = kwargs.get('move_arm_server', '/move_arm_server')
        self.move_arm_client = actionlib.SimpleActionClient(move_arm_server_name, MoveArmAction)
        rospy.loginfo('[push_door] Waiting for %s server', move_arm_server_name)
        if not self.move_arm_client.wait_for_server(rospy.Duration.from_sec(self.timeout)):
            raise RuntimeError("[push_door] timeout waiting for '{}' server".format(move_arm_server_name))

    def execute(self, userdata):
        self.say('Pushing open door')
        arm_goal = MoveArmGoal()
        arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        arm_goal.named_target = 'neutral'
        self.move_arm_client.send_goal(arm_goal)
        if not self.move_arm_client.wait_for_result(rospy.Duration.from_sec(self.timeout)):
            rospy.logerr('failed to put arm in neutral after {} seconds'.format(self.timeout))
            if self.number_of_retries > 0:
                self.number_of_retries -= 1
                return 'failed'
            return 'failed_after_retrying'

        goal = MoveForwardGoal()
        goal.movement_duration = self.movement_duration
        goal.speed = self.speed
        self.move_forward_client.send_goal(goal)
        if not self.move_forward_client.wait_for_result(rospy.Duration.from_sec(self.timeout)):
            if self.number_of_retries > 0:
                self.number_of_retries -= 1
                return 'failed'
            return 'failed_after_retrying'
        return 'succeeded'
