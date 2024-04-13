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
import moveit_commander

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
        self.device_part_name = "Razer Barracuda X"

        # Initialize the microphone and adjust for ambient noise
        self.init_microphone()

        self.bridge = CvBridge()
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        # self.person_img_sub = rospy.Subscriber('/annotated_image',Image, callback=self.callback)
        self.person_img_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw',Image, callback=self.callback)
        self.annotated_image = None
        self.face_image = None
        self.cropped_face_img = None
        self.image_rgb = None
        self.user_list=None
        
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
        self.listening_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Listening_Image.jpg'
        self.processing_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Processing_Image.jpg'
        self.error_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Error_Image.jpg'
        self.idle_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Idle_Ellipsis_Image.jpg' 
        self.device_index = self.find_device_index()
        rospy.logerr('Initialising moveit group')
        self.head = moveit_commander.MoveGroupCommander("head")
        rospy.logerr('Initialised moveit group')

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)
        
    def find_device_index(self):
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            if self.device_part_name in name:
                return index
        return None
    
    def update_display_with_image(self, display_type):
        
        if display_type == 'listening':
            img_path = self.listening_img_path
        elif display_type == 'processing':
            img_path = self.processing_img_path
        elif display_type == 'idle':
            img_path = self.idle_img_path
        elif display_type == 'error':
            img_path = self.error_img_path
        else:
            rospy.loginfo("Unknow display type: {}".format(display_type))
        img = cv2.imread(img_path)
        image_message =self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(image_message)

    def init_microphone(self):
        device_index = self.find_device_index()
        self.mic_source = sr.Microphone(device_index=device_index)
        with self.mic_source as source:
            self.r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Microphone is set up and ambient noise level adjusted.")

    def listen_and_transcribe(self):
        rospy.loginfo("Listening...")
        self.update_display_with_image('listening')
        with self.mic_source as source:
            audio = self.r.listen(source)

        self.update_display_with_image('processing')
        try:
            recognized_text = self.r.recognize_google(audio)
            self.update_display_with_image('idle')
            return recognized_text
        except (sr.UnknownValueError, sr.RequestError) as e:
            rospy.loginfo(f"Speech recognition error: {e}")
            self.update_display_with_image('error')
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
            self.say_this("Face not detected. Taking another Image. Stand infront of me.")
            rospy.sleep(3)
            self.get_face(self.annotated_image)

    def process_image_1(self, image_rgb, face_img):
        facial_feature = face_img[0]['facial_area']
        face_bb = [facial_feature['x'], facial_feature['y'], facial_feature['w'], facial_feature['h']]
        bbox = (face_bb[0], face_bb[1], face_bb[2]+face_bb[0], face_bb[3]+face_bb[1])
        # cv2.rectangle(saad_img, (face_bb[0], face_bb[1]), (face_bb[2]+face_bb[0], face_bb[3]+face_bb[1]), (255, 0, 0), 2)
        cropped_img = image_rgb[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        return cropped_img
    
    def execute(self, userdata):

        self.tilt_angle = 0.0
        self.head.set_joint_value_target("head_tilt_joint", self.tilt_angle)
        self.head.go()
        
        rospy.loginfo("Initiating receptionist interaction...")

        rospy.sleep(2)  # Small delay before greeting

        # Initial greeting
        self.say_this("Hello, I am Lucy, here to welcome you. Please stand still")
        rospy.sleep(3)

        # self.image_rgb, self.face_image = self.get_face(self.annotated_image)
        # self.cropped_face_img = self.process_image_1(self.image_rgb, self.face_image)
        # cv2.imwrite("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/faceImage_1.png", self.cropped_face_img)
        # self.person_data["person_image"]=self.cropped_face_img
        
        # Get the  name
        while not self.person_data['guest_name']: #checking if the name is still empty string
            self.say("May I know your name?")
            name_response = self.listen_and_transcribe()
            rospy.loginfo(f"{name_response}")
            if name_response:
                guest_name = self.extract_information_spacy(name_response, nlp_name)
                if guest_name:
                    self.person_data["guest_name"] = guest_name
                else:
                    self.say_this("I could not understand your name, can you please repeat")
            else:
                self.say_this("I did not hear your name, Please say it louder")
                
        self.image_rgb, self.face_image = self.get_face(self.annotated_image)
        self.cropped_face_img = self.process_image_1(self.image_rgb, self.face_image)
        cv2.imwrite("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/faceImage_1.png", self.cropped_face_img)
        self.person_data["person_image"]=self.cropped_face_img
        
        # Get the drink name 
        while not self.person_data['favorite_drink']:
            self.say_this("What is your favorite drink?")
            drink_response = self.listen_and_transcribe()
            if drink_response:
                favorite_drink = self.extract_information_spacy(drink_response, nlp_drink)
                if favorite_drink:
                    self.person_data["favorite_drink"] = favorite_drink
                else:
                    self.say_this("I could not identify your favorite drink,  can you please repeat")
            else:
                self.say_this("I did not hear your response, Please say it louder")

        userdata.persons_record=[self.person_data]

        self.say_this(f"Welcome, {guest_name}. I have noted that your favorite drink is {favorite_drink}. Now, please follow me to the sitting area.")
        rospy.loginfo(self.person_data)
    
        rospy.sleep(5)
        return 'succeeded'

