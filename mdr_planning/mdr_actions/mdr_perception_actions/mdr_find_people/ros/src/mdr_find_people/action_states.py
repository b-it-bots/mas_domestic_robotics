import rospy
import smach

from mdr_find_people.msg import FindPeopleResult
from mcr_perception_msgs.msg import Person, PersonList
from mas_perception_libs import ImageDetectionKey
from find_people import FindPeople


class FindPeopleState(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['find_people_goal'],
                             output_keys=['find_people_result', 'error_message'])

	self._listener = tf.TransformListener()

    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_PEOPLE')
        
        predictions, bb2ds, poses = FindPeople.single_shot_detection()


        # Just for debugging
        image = FindPeople.render_image_with_detections(cloud_msg, bb2ds)


        pl = []

        for i in range(len(predictions)):
            p = Person()
            p.id = i
            p.probability = predictions[i][ImageDetectionKey.CONF]

            pl.append(p)

        person_list = PersonList()
        person_list.persons = pl
        result = FindPeopleResult()
        result.person_list = person_list

        userdata['find_people_result'] = result
        return 'succeeded'
