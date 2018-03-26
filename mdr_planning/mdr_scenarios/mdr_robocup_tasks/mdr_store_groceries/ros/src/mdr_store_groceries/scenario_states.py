import rospy
import smach
from actionlib import SimpleActionClient

from mdr_move_base.msg import MoveBaseAction, MoveBaseGoal

class MoveBase(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.destination_locations = list(kwargs.get('destination_locations', list()))
        self.timeout = kwargs.get('timeout', 120.)
        self.action_server = kwargs.get('action_server', '/mdr_actions/move_base_server')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

        self.client = SimpleActionClient(self.action_server, MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        for i, destination_location in enumerate(self.destination_locations):
            goal.destination_location = destination_location

            rospy.loginfo('Sending the base to %s' % destination_location)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))

            res = self.client.get_result()
            if res and res.success:
                rospy.loginfo('Successfully reached %s' % destination_location)
            else:
                rospy.logerr('Could not reach %s' % destination_location)
                if self.retry_count == self.number_of_retries:
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        return 'succeeded'

class PerceiveTable(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        success = True
        # TODO: call an action for perceiving the table
        if success:
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'

class Pick(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        success = True
        # TODO: call an action for picking an object
        if success:
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'

class Place(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        success = True
        # TODO: call an action for placing an object
        if success:
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'
