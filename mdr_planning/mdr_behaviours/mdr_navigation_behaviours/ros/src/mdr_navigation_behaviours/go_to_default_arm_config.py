import time
import rospy
import actionlib

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from actionlib_msgs.msg import GoalStatus
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class GoToDefaultArmConfig(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'go_to_default_arm_config',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying']
                                   )
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'go_to_default_arm_config')
        self.timeout = kwargs.get('timeout', 60.)
        self.safe_arm_joint_config = kwargs.get('safe_arm_joint_config','neutral')
        self.move_arm_server=kwargs.get('move_arm_server','move_arm_server')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[move_base] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[move_base] %s server does not seem to respond', self.move_arm_server)

    def execute(self, userdata):
        rospy.loginfo('[move_base] Moving the arm to a safe configuration...')
        self.say('Moving the arm to a safe configuration')
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = MoveArmGoal.JOINT_VALUES
        move_arm_goal.named_target = self.safe_arm_joint_config
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()

        if self.move_arm_client.wait_for_result(rospy.Duration.from_sec(self.timeout)):
            result = self.move_arm_client.get_result()
            if result == GoalStatus.SUCCEEDED:
                rospy.loginfo('Successfully reached safe configuration')
            else:
                rospy.logerr('Could not reach safe configuration')
                if self.retry_count == self.number_of_retries:
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        return 'succeeded'

