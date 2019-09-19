import rospy
import actionlib
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal


class CloseDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'close_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'close_door')
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
        self.say('trying to close door')
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
