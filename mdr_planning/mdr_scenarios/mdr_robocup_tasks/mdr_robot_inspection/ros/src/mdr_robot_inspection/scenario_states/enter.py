import rospy
import smach
from actionlib import SimpleActionClient
from mdr_enter_door_action.msg import EnterDoorAction, EnterDoorGoal

class Enter(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                             output_keys=['enter_door_feedback'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.enter_action_client = SimpleActionClient(kwargs.get('enter_action_server',
                                                                 '/mdr_actions/enter_door_server'),
                                                      EnterDoorAction)
        self.enter_action_client.wait_for_server()

    def execute(self, userdata):
        goal = EnterDoorGoal()
        rospy.loginfo('[ENTER_DOOR] Calling door entering action')
        self.enter_action_client.send_goal(goal)
        duration = rospy.Duration.from_sec(self.timeout)
        success = self.enter_action_client.wait_for_result(duration)
        if success:
            rospy.loginfo('Entered successfully')
            return 'succeeded'
        else:
            rospy.logerr('Entering failed')
            if self.retry_count == self.number_of_retries:
                return 'failed_after_retrying'
            return 'failed'
