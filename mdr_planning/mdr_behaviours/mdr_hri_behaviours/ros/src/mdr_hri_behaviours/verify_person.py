"""
Receptionist Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad, Ayusee Swain
zainey4@gmail.com
"""
import rospy
import random
import spacy
import speech_recognition as sr
from polyglot.text import Text
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import String  # ROS standard string message
from threading import Thread #import the threading module for running background task
from deepface import DeepFace
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from deepface.modules import verification
from deepface.models.FacialRecognition import FacialRecognition
from deepface.commons.logger import Logger
import matplotlib.pyplot as plt


# Load SpaCy models
nlp_drink = spacy.load("/home/lucy/rasa_ws/spacy_model/model_drink")  # Model trained to identify drinks
nlp_name = spacy.load("/home/lucy/rasa_ws/spacy_model/model_name")  # Model trained to identify names

class VerifyPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receptionist_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['persons_record'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)
        self.person_data=None
        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.bridge = CvBridge()
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        self.person_img_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw',Image, callback=self.callback)
        self.annotated_image = None
        self.face_image = None
        self.image_rgb = None
        self.person_verify = None
        self.verified_guest_name = None
        self.verified_guest_drink = None

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def listen_and_transcribe(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Listening...")
            audio = self.r.listen(source)
        try:
            return self.r.recognize_google(audio)
        except (sr.UnknownValueError, sr.RequestError):
            rospy.loginfo("I am sorry, I did not catch that. Could you please repeat?")
            return None

    def extract_information_spacy(self, sentence, model):
        try:
            doc = model(sentence)
            entities = [ent.text for ent in doc.ents]
            return entities[0] if entities else None
        except Exception as e:
            print(f"Error processing sentence with SpaCy: {e}")
            return None

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.annotated_image = cv_image

    def get_face(self, image):
        try:
            face_img = DeepFace.extract_faces(image)
            return image, face_img
        except ValueError as e:
            rospy.loginfo("Face not detected")
            # self.say_this("Face not detected. Taking another Image. Stand infront of me.")
            rospy.sleep(3)
            self.get_face(self.annotated_image)

    def process_image_1(self, image_rgb, face_img):
        # Convert image to RGB
        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # face_img = DeepFace.extract_faces(image_rgb)
        facial_feature = face_img[0]['facial_area']
        face_bb = [facial_feature['x'], facial_feature['y'], facial_feature['w'], facial_feature['h']]
        # print(facial_feature)
        bbox = (face_bb[0], face_bb[1], face_bb[2]+face_bb[0], face_bb[3]+face_bb[1])
        # cv2.rectangle(saad_img, (face_bb[0], face_bb[1]), (face_bb[2]+face_bb[0], face_bb[3]+face_bb[1]), (255, 0, 0), 2)
        cropped_img = image_rgb[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        return cropped_img



        
    

    #Face recognition

    # def process_image(self, img_path):
    #     img = DeepFace.extract_faces(img_path=img_path, target_size=self.target_size)[0]["face"]
    #     img = np.expand_dims(img, axis=0)  # Shape: (1, height, width, 3)
    #     img_representation = self.model.find_embeddings(img)
    #     return img, img_representation

    # def calculate_distance(self, img1_representation, img2_representation):
    #     distance_vector = np.square(img1_representation - img2_representation)
    #     return np.sqrt(distance_vector.sum())

    # def compare_faces(self):
    #     current_distance = self.calculate_distance(self.img2_representation, self.img2_representation)
    #     threshold = verification.find_threshold(model_name=self.model_name, distance_metric="euclidean")
    #     self.logger.info(f"Euclidean distance: {current_distance}")
    #     self.logger.info(f"Threshold for {self.model_name}-euclidean pair is {threshold}")

    #     if current_distance < threshold:
    #         self.logger.info(
    #             f"This pair is the same person because its distance {current_distance}"
    #             f" is less than the threshold {threshold}"
    #         )
    #     else:
    #         self.logger.info(
    #             f"This pair is different persons because its distance {current_distance}"
    #             f" is greater than the threshold {threshold}"
    #         )

    # def plot_results(self, img1, img1_representation, img2, img2_representation, distance_vector):
    #     fig = plt.figure()
    #     # Add subplots for each image, their representations, and the distance
    #     # Assuming img1 and img2 are already expanded as needed for plotting
    #     # Your plotting code here, similar to what you already have
        
    #     plt.show()

    
    def execute(self, userdata):
        
        rospy.loginfo("Initiating person verification task...")
        self.person_data=userdata.persons_record
        rospy.sleep(2)  # Small delay before greeting
        
        introduction_phrase = "Hello! Please stand infront of me so that I can verify your identity"
        self.say_this(introduction_phrase)
        rospy.sleep(3)

        self.image_rgb, self.face_image = self.get_face(self.annotated_image)
        self.cropped_face_img = self.process_image_1(self.image_rgb, self.face_image)
        self.say_this("Image Captured!!")
        # if self.face_image:
        #     #Repeat self.process_image_1


        for user_dic in self.person_data:
            cv2.imwrite("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/test.png", self.cropped_face_img)
            self.person_verify = DeepFace.verify(self.cropped_face_img, user_dic['person_image'])
            if self.person_verify['verified']:
                self.verified_guest_name = user_dic['guest_name']
                self.verified_guest_drink = user_dic['favorite_drink']

        # result = DeepFace.verify(self.face_image, saad_2)


        # Initial greeting
        if not self.person_verify['verified']:
            self.say_this("I have no information about this guest")
            return 'failed'
        else:
            self.say_this(f"Greetings {self.verified_guest_name}, I have verified you in the guest list. Your favourite drink is {self.verified_guest_drink}")
            rospy.sleep(5)
            return 'succeeded'


