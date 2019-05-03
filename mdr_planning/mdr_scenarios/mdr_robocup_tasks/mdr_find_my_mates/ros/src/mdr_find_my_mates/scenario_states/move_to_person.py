import rospy
import actionlib
from mcr_perception_msgs.msg import Person
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class MoveToPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'move_to_person',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'move_to_person')
        self.move_base_action_server_name = kwargs.get('move_base_action_server_name',
                                                       'move_base_server')
        self.move_base_timeout = kwargs.get('move_base_timeout', 15.)

        self.move_base_client = actionlib.SimpleActionClient(self.move_base_action_server_name,
                                                             MoveBaseAction)
        rospy.loginfo('[move_to_person] Waiting for %s', self.move_base_action_server_name)
        self.move_base_client.wait_for_server()

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        unknown_people = self.kb_interface.get_all_attributes('unknown')
        people_identifiers = []
        for item in unknown_people:
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'person':
                        people_identifiers.append(param.value)

        print(people_identifiers)
        if not people_identifiers:
            print('[move_to_person] No people found; aborting operation')
            return 'failed_after_retrying'

        person_to_interview = people_identifiers[0]
        person_info = self.kb_interface.get_obj_instance(person_to_interview, Person)
        person_pose = person_info.safe_pose

        move_base_goal = MoveBaseGoal()
        move_base_goal.goal_type = MoveBaseGoal.POSE
        move_base_goal.pose = person_pose

        rospy.loginfo('[move_to_person] Going to person ' + person_to_interview +
                      ' at pose ' + str(person_pose.pose.position.x) + ' ' +
                      str(person_pose.pose.position.y))
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(self.move_base_timeout)))
        result = self.move_base_client.get_result()
        if result and result.success:
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            rospy.loginfo('[move_to_person] Could not go to %s', person_to_interview)
            self.say('[move_to_person] Could not go to %s; aborting operation', person_to_interview)
            return 'failed_after_retrying'
        rospy.logerr('[move_to_person] Could not go to %s; retrying', person_to_interview)
        self.retry_count += 1
        return 'failed'
