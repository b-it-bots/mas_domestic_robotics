import rospy
import actionlib

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class MoveBase(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'move_base',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                                   input_keys=['destination_locations'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'move_base')
        self.action_server_name = kwargs.get('action_server_name', 'move_base_server')
        self.destination_locations = list(kwargs.get('destination_locations', None))
        self.timeout = kwargs.get('timeout', 120.)
        self.debug = kwargs.get('debug', False)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.destination_locations is None:
            self.destination_locations = userdata.destination_locations
            rospy.loginfo("Using userdata's destination_locations %s", self.destination_locations)

        client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)
        client.wait_for_server()
        for destination_location in self.destination_locations:
            goal = MoveBaseGoal()
            goal.goal_type = MoveBaseGoal.NAMED_TARGET
            goal.destination_location = destination_location

            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            result = client.get_result()
            if result:
                if result.success:
                    rospy.loginfo('[%s] Successfully moved to %s',
                                  self.state_name, destination_location)
                else:
                    rospy.logerr('[%s] Could not move to %s', self.state_name, destination_location)
                    if self.retry_count == self.number_of_retries:
                        rospy.logerr('[%s] Could not move to %s even after retrying; giving up',
                                     self.state_name, destination_location)
                        self.retry_count = 0
                        return 'failed_after_retrying'
                    self.retry_count += 1
                    return 'failed'
            else:
                rospy.logerr('[%s] Could not move to %s within %f seconds; giving up',
                             self.state_name, destination_location, self.timeout)
                client.cancel_all_goals()
        return 'succeeded'
