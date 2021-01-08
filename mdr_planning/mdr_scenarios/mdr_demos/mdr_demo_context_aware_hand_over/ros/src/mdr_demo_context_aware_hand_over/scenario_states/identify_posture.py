import numpy as np

import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class IdentifyPosture(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'identify_posture',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['person_list'],
                                   output_keys=['posture', 'person_pose'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_context_aware_hand_over')
        self.posture_height_width_ratio_ranges = kwargs.get('posture_height_width_ratio_ranges', {})

    def execute(self, userdata):
        self.say('I will hand over the object to the closest person')

        closest_person_idx = find_closest_person(userdata.person_list.persons)
        if not userdata.person_list.persons[closest_person_idx].views:
            rospy.loginfo('[identify_posture] Failed to get views of person ')
            return 'failed'

        first_person_image_bb_width = float(userdata.person_list.persons[closest_person_idx].views[0].image.width)
        first_person_image_bb_height = float(userdata.person_list.persons[closest_person_idx].views[0].image.height)
        bb_height_width_ratio = first_person_image_bb_height / first_person_image_bb_width
        rospy.loginfo('[identify_posture] Height width bb ratio: {0}'.format(bb_height_width_ratio))

        person_posture = None
        for posture, ratio_ranges in self.posture_height_width_ratio_ranges.items():
            if ratio_ranges[0] <= bb_height_width_ratio <= ratio_ranges[1]:
                person_posture = posture

        userdata.person_pose = userdata.person_list.persons[closest_person_idx].pose

        rospy.loginfo('[identify_posture] Found a {0} person'.format(person_posture))
        userdata.posture = person_posture
        return 'succeeded'


def find_closest_person(people):
    '''Returns the index of the person closest to the robot.

    Keyword arguments:
    people: Sequence[mas_perception_msgs.msg.Person] -- list of people in the current scene

    '''
    distances = np.zeros(len(people))
    for i, person in enumerate(people):
        distances[i] = np.linalg.norm([person.pose.pose.position.x,
                                       person.pose.pose.position.y,
                                       person.pose.pose.position.z])
    return np.argmin(distances)
