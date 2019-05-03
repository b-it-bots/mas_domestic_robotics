import rospy
from std_msgs.msg import String
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_perception_msgs.msg import PersonInfo

class Report(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'report',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['command'])
        self.timeout = kwargs.get('timeout', 120.)

    def execute(self, userdata):
        rospy.loginfo('Describing my mates ...')

        known_people = self.kb_interface.get_all_attributes('known')
        people_names = []
        for item in known_people:
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'person':
                        people_names.append(param.value)

        for name in people_names:
            info_msg = self.kb_interface.get_obj_instance(name, PersonInfo)
            self.say(info_msg.name + 'is wearing a ' + info_msg.clothes_colour + ' shirt.')
            if info_msg.gender == 'male':
                self.say('He is on the ' + info_msg.location)
            else:
                self.say('She is on the ' + info_msg.location)
        return 'succeeded'
