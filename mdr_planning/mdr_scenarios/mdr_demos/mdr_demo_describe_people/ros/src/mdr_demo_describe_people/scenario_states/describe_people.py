import rospy
import actionlib

from sensor_msgs.msg import Image
from mdr_detect_person.msg import DetectPersonAction, DetectPersonGoal
from mdr_recognize_emotion_action.msg import RecognizeEmotionAction, RecognizeEmotionGoal
from mdr_gender_recognition.msg import GenderRecognitionAction, GenderRecognitionGoal

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class DescribePeople(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'describe_people',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'no_image_received'])

        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 10.))
        self.image_topic = kwargs.get('image_topic', '/image')
        self.detect_person_server = kwargs.get('detect_person_server',
                                               '/mdr_actions/detect_person_server')
        self.recognize_emotion_server = kwargs.get('recognize_emotion_server',
                                                   '/mdr_actions/recognize_emotion_server')
        self.recognize_gender_server = kwargs.get('recognize_gender_server',
                                                  '/mdr_actions/gender_recognition_server')
        self.sound_topic = kwargs.get('sound_topic', '/say')

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.get_image)
        self.image_received = False
        self.image = None

        self.detect_person_client = actionlib.SimpleActionClient(self.detect_person_server,
                                                                 DetectPersonAction)
        rospy.loginfo('Waiting for %s' % self.detect_person_server)
        self.detect_person_client.wait_for_server()

        self.recognize_emotion_client = actionlib.SimpleActionClient(self.recognize_emotion_server,
                                                                     RecognizeEmotionAction)
        rospy.loginfo('Waiting for %s' % self.recognize_emotion_server)
        self.recognize_emotion_client.wait_for_server()

        self.recognize_gender_client = actionlib.SimpleActionClient(self.recognize_gender_server,
                                                                    GenderRecognitionAction)
        rospy.loginfo('Waiting for %s' % self.recognize_gender_server)
        self.recognize_gender_client.wait_for_server()

        rospy.loginfo('Starting people description')
        self.start_time = rospy.Time.now()

    def execute(self, userdata):
        while not self.image_received and (rospy.Time.now() - self.start_time) < self.timeout:
            rospy.sleep(0.1)

        if self.image is None:
            self.say('Something is wrong with my camera; I cannot see anything.')
            rospy.logerr('Could not receive image')
            return 'no_image_received'

        # detecting people
        detect_person_goal = DetectPersonGoal()
        detect_person_goal.start = True
        detect_person_goal.image = self.image

        rospy.loginfo('Detecting people...')
        self.detect_person_client.send_goal(detect_person_goal)
        self.detect_person_client.wait_for_result()
        detect_person_result = self.detect_person_client.get_result()

        face_count = detect_person_result.number_of_faces
        if face_count == 0:
            self.say('I could not see anyone. Could you please stand in front of my camera?')
            rospy.logerr('Could not find any people in the image; retrying')
            return 'failed'

        # recognizing emotions
        emotion_goal = RecognizeEmotionGoal()
        emotion_goal.image = self.image
        emotion_goal.number_of_faces = face_count
        emotion_goal.bounding_boxes = detect_person_result.bounding_boxes

        rospy.loginfo('Recognizing emotions...')
        self.recognize_emotion_client.send_goal(emotion_goal)
        self.recognize_emotion_client.wait_for_result()
        emotion_result = self.recognize_emotion_client.get_result()

        # recognizing gender
        rospy.loginfo('Recognizing genders...')
        gender_goal = GenderRecognitionGoal()
        gender_goal.image = self.image
        gender_goal.number_of_faces = face_count
        gender_goal.bounding_boxes = detect_person_result.bounding_boxes

        self.recognize_gender_client.send_goal(gender_goal)
        self.recognize_gender_client.wait_for_result()
        gender_result = self.recognize_gender_client.get_result()

        # say how many people there are
        sentence = ''
        if face_count == 1:
            sentence = 'I see ' + str(face_count) + ' person'
        else:
            sentence = 'I see ' + str(face_count) + ' people'
        self.say(sentence)
        rospy.loginfo(sentence)

        # say the emotion and gender of the first person
        sentence = 'I see one ' + str(emotion_result.emotions[0]) + ' ' \
                   + str(gender_result.genders[0])
        self.say(sentence)
        rospy.loginfo(sentence)

        # say the emotion and gender of the other people (except for the last one)
        # adding 'also' to the sentence for a more engaging feedback
        for i in xrange(1, face_count-1):
            sentence = 'I also see one ' + str(emotion_result.emotions[i]) + ' ' \
                        + str(gender_result.genders[i])
            self.say(sentence)
            rospy.loginfo(sentence)

        # if more than two people have been detected, say the emotion and gender
        # of the last person, but add 'finally' to the sentence for a more engaging feedback
        if face_count > 2:
            sentence = 'Finally, I see one ' + str(emotion_result.emotions[face_count-1]) \
                       + ' ' + str(gender_result.genders[face_count-1])
            self.say(sentence)
            rospy.loginfo(sentence)

        self.image_received = False
        self.image = None
        return 'succeeded'

    def get_image(self, image_msg):
        self.image = image_msg
        self.image_received = True
