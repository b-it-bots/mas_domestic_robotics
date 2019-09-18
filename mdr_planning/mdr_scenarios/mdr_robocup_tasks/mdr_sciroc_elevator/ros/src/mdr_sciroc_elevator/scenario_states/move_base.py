import rospy
import actionlib
from std_msgs.msg import String

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class MoveBase(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'move_base',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', 'mdr_sciroc_elevator')
        self.state_name = kwargs.get('state_name', 'move_base')
        self.action_server = kwargs.get('action_server', 'move_base_server')
        self.destination_locations = list(kwargs.get('destination_locations', list()))
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.client = actionlib.SimpleActionClient(self.action_server, MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10.))

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        goal = MoveBaseGoal()
        for destination_location in self.destination_locations:
            goal.destination_location = destination_location

            # calling the actionlib server and waiting for the execution to end
            rospy.loginfo('[move_base] Sending action lib goal to move_base_server,' +
                          ' destination: ' + goal.destination_location)
            self.say('Going to ' + goal.destination_location)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            success = self.client.get_result()

            if success:
                rospy.loginfo('Successfully reached %s' % destination_location)
            else:
                rospy.logerr('Could not reach %s' % destination_location)
                self.say('Could not reach ' + goal.destination_location)
                if self.retry_count == self.number_of_retries:
                    self.say('Aborting operation')
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        return 'succeeded'
