import rospy
import actionlib
import moveit_commander
from mdr_find_people.msg import FindPeopleAction, FindPeopleGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class DetectPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_person_front',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                                   output_keys=['person_list'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_context_aware_hand_over')
        self.action_server = kwargs.get('action_server', 'find_people_server')
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.client = actionlib.SimpleActionClient(self.action_server, FindPeopleAction)
        rospy.logerr(f'Waiting for server {self.action_server}')
        self.client.wait_for_server(rospy.Duration(10.))

        rospy.logerr('Initialising moveit group')
        self.head=moveit_commander.MoveGroupCommander("head")
        rospy.logerr('Initialised moveit group')

        self.tilt_angle=0.2

    def execute(self, userdata):      
        self.head.set_joint_value_target("head_tilt_joint", self.tilt_angle)
        self.head.go()

        goal = FindPeopleGoal()
        # calling the actionlib server and waiting for the execution to end
        rospy.loginfo('[detect_person] Sending action lib goal to {0}'.format(self.action_server))
        self.say('Detecting people')
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = self.client.get_result()

        if result and result.person_list.persons:
            rospy.loginfo('[detect_person] Found {0} people'.format(len(result.person_list.persons)))
            userdata.person_list = result.person_list
            return 'succeeded'
        else:
            rospy.logerr('Could not detect a person')
            self.say('Could not detect a person')
            if self.retry_count == self.number_of_retries:
                self.say('Aborting operation')
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'failed'
