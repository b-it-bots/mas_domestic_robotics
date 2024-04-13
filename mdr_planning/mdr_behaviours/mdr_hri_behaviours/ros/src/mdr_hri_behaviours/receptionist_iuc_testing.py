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

class ReceptionistTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receptionist_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['persons_record'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)
        self.person_data={'guest_name':'', 'favorite_drink':'', 'person_image':None} ## list of string, list of string, list of ndarray
        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.bridge = CvBridge()
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        # self.person_img_sub = rospy.Subscriber('/annotated_image',Image, callback=self.callback)
        self.person_img_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw',Image, callback=self.callback)
        self.annotated_image = None
        self.face_image = None
        self.cropped_face_img = None
        self.image_rgb = None
        self.user_list=None
        self.model = DeepFace.build_model(model_name="VGG-Face")
        self.target_size = self.model.input_shape



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

    # def get_face(self, image):
    #     try:
    #         face_img = DeepFace.extract_faces(image)
    #         return image, face_img
    #     except ValueError as e:
    #         rospy.loginfo("Face not detected")
    #         self.say_this("Face not detected. Taking another Image. Stand infront of me.")
    #         rospy.sleep(3)
    #         self.get_face(self.annotated_image)

    def get_face(self, max_retries=3):
        retry_count = 0
        while retry_count < max_retries:
            image = self.annotated_image
            try:
                face_imgs = DeepFace.extract_faces(image, target_size=self.target_size)[0]["face"]  # Specifying backend for consistency
                
                rospy.loginfo("Face detected successfully.")
                return face_imgs  # Return the first detected face for simplicity
                
            except Exception as e:
                rospy.loginfo(f"Error detecting face: {e}")
            rospy.sleep(3)  # Wait before retrying
            retry_count += 1

    
    # def get_face(self, max_retries=3):
    #     retry_count = 0
    #     while retry_count < max_retries:
    #         image = self.annotated_image
    #         try:
    #             face_imgs = DeepFace.extract_faces(image)  # Specifying backend for consistency
                
    #             rospy.loginfo("Face detected successfully.")
    #             return image, face_imgs  # Return the first detected face for simplicity
                
    #         except Exception as e:
    #             rospy.loginfo(f"Error detecting face: {e}")
    #         rospy.sleep(3)  # Wait before retrying
    #         retry_count += 1

    # def process_image_1(self, image_rgb, face_img):
    #     # Convert image to RGB
    #     # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     # face_img = DeepFace.extract_faces(image_rgb)
    #     facial_feature = face_img[0]['facial_area']
    #     face_bb = [facial_feature['x'], facial_feature['y'], facial_feature['w'], facial_feature['h']]
    #     # print(facial_feature)
    #     bbox = (face_bb[0], face_bb[1], face_bb[2]+face_bb[0], face_bb[3]+face_bb[1])
    #     # cv2.rectangle(saad_img, (face_bb[0], face_bb[1]), (face_bb[2]+face_bb[0], face_bb[3]+face_bb[1]), (255, 0, 0), 2)
    #     cropped_img = image_rgb[bbox[1]:bbox[3], bbox[0]:bbox[2]]
    #     return cropped_img
            

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

    def plot_results(self, img1, img1_representation, img2, img2_representation, distance_vector):
        fig = plt.figure()
        # Add subplots for each image, their representations, and the distance
        # Assuming img1 and img2 are already expanded as needed for plotting
        # Your plotting code here, similar to what you already have
        
        plt.show()

    
    def execute(self, userdata):
        
        rospy.loginfo("Initiating receptionist interaction...")

        rospy.sleep(2)  # Small delay before greeting

        # Initial greeting
        self.say_this("Hello, I am Lucy, here to welcome you. May I know your name?")
        rospy.sleep(2)
        name_response = self.listen_and_transcribe()

        # self.image_rgb, self.face_image = self.get_face()
        # self.cropped_face_img = self.process_image_1(self.image_rgb, self.face_image)
        self.face_image = self.get_face()
        # self.cropped_face_img = self.process_image_1(self.image_rgb, self.face_image)
        
        # self.face_image = self.process_image_1(self.annotated_image)
        # if self.face_image is not None:
        #     rospy.loginfo("Captured face image of guest...")
        #     rospy.loginfo(f"image captured = {self.face_image}")
        # else:
        #     rospy.loginfo("Could not capture the face image ...")
        cv2.imwrite("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/test_1.png", self.cropped_face_img)
        self.person_data["person_image"]=self.face_image

        rospy.loginfo(f"{name_response}")
        if name_response:
            guest_name = self.extract_information_spacy(name_response, nlp_name)
            if guest_name:
                self.person_data["guest_name"]=guest_name
                # userdata.guest_name = guest_name
            else:
                self.say_this("I could not identify your name correctly. Let's try again.")
                return 'failed'
        else:
            return 'failed'

        self.say_this("What is your favorite drink?")
        drink_response = self.listen_and_transcribe()
        if drink_response:
            favorite_drink = self.extract_information_spacy(drink_response, nlp_drink)
            if favorite_drink:
                # userdata.favorite_drink = favorite_drink
                self.person_data["favorite_drink"]=favorite_drink
            else:
                self.say_this("I could not identify your favorite drink correctly. Let's try again.")
                return 'failed'
        else:
            return 'failed'

        # # Check if the list already exists
        # if hasattr(userdata, 'persons_record'):
        #     # If it exists, append the new data
        #     userdata.persons_record.append(self.person_data)
        # else:
        #     # If it doesn't exist, initialize it with the new data as the first item
        #     userdata.persons_record = [self.person_data]

        userdata.persons_record=[self.person_data]

        self.say_this(f"Welcome, {guest_name}. I have noted that your favorite drink is {favorite_drink}. Now, please follow me to the sitting area.")
        rospy.loginfo(self.person_data)
    
        rospy.sleep(5)
        return 'succeeded'


