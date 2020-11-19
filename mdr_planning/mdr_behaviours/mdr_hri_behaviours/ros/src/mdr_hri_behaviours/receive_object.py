import rospy
import actionlib

from mas_perception_msgs.msg import Person
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_receive_object_action.msg import ReceiveObjectAction, ReceiveObjectGoal

class ReceiveObject(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receive_object',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying',
                                             'person_not_found'],
                                   input_keys=['person_name'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'get_person_pose')
        self.action_server = kwargs.get('action_server', 'receive_object_server')
        self.timeout = kwargs.get('timeout', 120.)
        self.context_aware = kwargs.get('context_aware', True)
        self.posture_ratio_ranges = kwargs.get('posture_ratio_ranges', {})
        self.person_name = kwargs.get('person_name', None)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.client = actionlib.SimpleActionClient(self.action_server, ReceiveObjectAction)
        self.client.wait_for_server(rospy.Duration(10.))

    def execute(self, userdata):
        person_to_look_for = self.person_name
        if not person_to_look_for:
            person_to_look_for = userdata.person_name

        person_msg = self.kb_interface.get_obj_instance(person_to_look_for, Person._type)
        if person_msg is None:
            return 'person_not_found'

        posture = self.get_posture(person_msg)
        reception_action_result = self.receive_object(person_msg, posture)

        if reception_action_result and reception_action_result.success:
            rospy.loginfo('Received object')
            return 'succeeded'
        else:
            rospy.logerr('Could not receive object')
            self.say('Object not received')
            if self.retry_count == self.number_of_retries:
                self.say('Aborting operation')
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'failed'
        return 'succeeded'

    def get_posture(self, person_msg):
        person_image_bb_width = float(person_msg.views[0].image.width)
        person_image_bb_height = float(person_msg.views[0].image.height)
        bb_height_width_ratio = person_image_bb_height / person_image_bb_width

        person_posture = None
        for posture, ratio_ranges in self.posture_ratio_ranges.items():
            if ratio_ranges[0] <= bb_height_width_ratio <= ratio_ranges[1]:
                person_posture = posture
        return person_posture

    def receive_object(self, person_msg, posture):
        goal = ReceiveObjectGoal()
        goal.posture_type = posture
        goal.context_aware = self.context_aware
        goal.reception_detection = self.reception_detection
        goal.person_pose = person_msg.pose

        # calling the actionlib server and waiting for the execution to end
        rospy.loginfo('[receive_object] Sending action lib goal to {0}'.format(self.action_server))
        self.say('Receiving object from a {0} person'.format(posture))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = self.client.get_result()
        return result
