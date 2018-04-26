import rospy
import smach
import smach_ros
from std_msgs.msg import String
from mdr_detect_person.msg import DetectPersonAction, DetectPersonGoal

from mdr_turn_base_to_action.msg import TurnBaseToAction, TurnBaseToGoal

from mdr_gender_recognition.msg import GenderRecognitionAction, \
                                       GenderRecognitionGoal

from sensor_msgs.msg import Image
from actionlib import SimpleActionClient


def say(publisher, message):
    msg = String()
    msg.data = message
    rospy.loginfo(msg.data)
    publisher.publish(msg)


class FindCrowd(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                             output_keys=['image', 'bounding_boxes',
                                          'number_of_faces'])

        self.timeout = kwargs.get('timeout', 3)
        self.image_topic = kwargs.get('image_topic', 'some/topic')
        self.say_topic = kwargs.get('say_topic', '/say')
        self.number_of_retries = kwargs.get('number_of_retries', 3)
        self.retry_count = 0
        self.image = None

        rospy.Subscriber(image_topic, Image, queue_size=10, image_cb)
        say_pub = rospy.Publisher(say_topic, String, queue_size=1)
        detect_person_client = SimpleActionClient('mdr_actions/detect_person_server',
                                                  DetectPersonAction)
        detect_person_client.wait_for_server()

        turn_base_client = SimpleActionClient('mdr_actions/turn_to_server',
                                              TurnToBaseAction)
        turn_base_client.wait_for_server()

    def execute(self, userdata):
        say(say_pub, 'I want to play riddles')
        rospy.sleep(self.timeout)

        # Turn 180
        turn_goal = TurnBaseToGoal()
        turn_goal.desired_yaw = 180.0
        turn_base_client.send_goal(turn_goal)

        # Detect person
        goal = DetectPersonGoal()
        goal.start = True
        goal.image = self.image
        userdata.image = goal.image

        detect_person_client.send_goal(goal)
        result = detect_person_client.wait_for_result()

        if result.number_of_faces > 0:
            # Say how many there are
            msg = String()
            msg.data = "I can see %i persons" % result.number_of_faces
            userdata.number_of_faces = result.number_of_faces
            userdata.bounding_boxes = result.bounding_boxes
            rospy.loginfo(msg.data)
            say_pub.publish(msg)
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to find crowd %s' % obj_to_grasp)
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to fild crowd %s' % obj_to_grasp)
        self.retry_count += 1
        # TODO Turn a bit? How to handle failures?
        return 'failed'

    def image_cb(self, msg):
        self.image = msg


class RecognizeGenders(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed']
                             input_keys=['image', 'number_of_faces',
                                         'bounding_boxes'])
        self.timeout = kwargs.get('timeout', 10)
        say_pub = rospy.Publisher(say_topic, String, queue_size=1)

        gender_client = SimpleActionClient('mdr_actions/gender_recognition_server',
                                           GenderRecognitionAction)
        gender_client.wait_for_server()

    def execute(self, userdata):
        # Recognize gender
        goal = GenderRecognitionGoal()
        goal.image = userdata.image
        goal.number_of_faces = userdata.number_of_faces
        goal.bounding_boxes = userdata.bounding_boxes
        gender_client.send_goal(goal)
        result = gender_client.wait_for_result()

        men = result.genders.count('man')
        women = result.genders.count('woman')

        # Say x men and y women
        msg = String()
        msg.data = 'There are %i men and %i women' % (men, women)
        rospy.loginfo(msg.data)
        say_pub.publish(msg)
        return 'succeeded'
