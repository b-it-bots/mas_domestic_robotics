import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class IdentifyPosture(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'identify_posture',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'],
                                   input_keys=['person_list'],
                                   output_keys=['posture'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_context_aware_hand_over')
        self.posture_height_width_ratio_ranges = kwargs.get('posture_height_width_ratio_ranges', {})

    def execute(self, userdata):
        self.say('I will only hand over the item to the first detected person')

        first_person_image_bb_width = float(userdata.person_list.persons[0].rgb_image.width)
        first_person_image_bb_height = float(userdata.person_list.persons[0].rgb_image.height)
        bb_height_width_ratio = first_person_image_bb_height / first_person_image_bb_width
        rospy.loginfo('[identify_posture] Height width bb ratio: {0}'.format(bb_height_width_ratio))

        person_posture = None
        for posture, ratio_ranges in self.posture_height_width_ratio_ranges.items():
            if ratio_ranges[0] <= bb_height_width_ratio <= ratio_ranges[1]:
                person_posture = posture

        rospy.loginfo('[identify_posture] Found a {0} person'.format(person_posture))
        userdata.posture = person_posture
        return 'succeeded'
