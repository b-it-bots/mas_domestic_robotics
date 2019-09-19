import rospy
import actionlib
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal


class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state, input_keys=['handle_pose'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

    def execute(self, userdata):
        self.say('trying to open door')
        rospy.loginfo('handle pose: {} {} {}'.format(userdata.handle_pose.pose.position.x,
                                                     userdata.handle_pose.pose.position.y,
                                                     userdata.handle_pose.pose.position.z))
        rospy.sleep(5.0)
        if self.number_of_retries > 0:
            self.number_of_retries -= 1
        else:
            return 'failed_after_retrying'
        return 'failed'


class AskToOpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'ask_to_open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'ask_to_open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

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
        self.timeout = kwargs.get('timeout', 25)

        # move forward
        self.movement_duration = float(rospy.get_param('~movement_duration', 5.))
        self.speed = float(rospy.get_param('~speed', 0.1))
        move_forward_server_name = rospy.get_param('~move_forward_server', '/move_forward_server')
        self.move_forward_client = actionlib.SimpleActionClient(move_forward_server_name, MoveForwardAction)
        rospy.loginfo('[push_door] Waiting for %s server', move_forward_server_name)
        if not self.move_forward_client.wait_for_server(rospy.Duration(self.timeout)):
            raise RuntimeError("[push_door] timeout waiting for '{}' server".format(move_forward_server_name))

        # move arm action
        move_arm_server_name = rospy.get_param('~move_arm_server', '/move_arm_server')
        self.move_arm_client = actionlib.SimpleActionClient(move_arm_server_name, MoveArmAction)
        rospy.loginfo('[push_door] Waiting for %s server', move_arm_server_name)
        if not self.move_arm_client.wait_for_server(rospy.Duration(self.timeout)):
            raise RuntimeError("[push_door] timeout waiting for '{}' server".format(move_arm_server_name))

    def execute(self, userdata):
        self.say('Pushing open door')
        arm_goal = MoveArmGoal()
        arm_goal.goal_type = MoveArmGoal.NAMED_TARGET
        arm_goal.named_target = 'neutral'
        self.move_arm_client.send_goal(arm_goal)
        if not self.move_arm_client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('failed to put arm in neutral after {} seconds'.format(self.timeout))
            if self.number_of_retries > 0:
                self.number_of_retries -= 1
                return 'failed'
            return 'failed_after_retrying'

        goal = MoveForwardGoal()
        goal.movement_duration = self.movement_duration
        goal.speed = self.speed
        self.move_forward_client.send_goal(goal)
        if not self.move_forward_client.wait_for_result(rospy.Duration(self.timeout)):
            if self.number_of_retries > 0:
                self.number_of_retries -= 1
                return 'failed'
            return 'failed_after_retrying'
        return 'succeeded'
