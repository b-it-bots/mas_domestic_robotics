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
from std_msgs.msg import String

import mediapipe as mp


mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

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
        # self.person_img_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw',Image, callback=self.callback)
        self.image_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/annotated_image', Image, queue_size=10)
        self.crop_pub = rospy.Publisher('/cropped_image', Image, queue_size=10)
        self.mp_pose = mp.solutions.pose
        self.annotated_image = None
        self.face_image = None
        self.cropped_face_img = None
        self.image_rgb = None
        self.user_list=None
        self.model = DeepFace.build_model(model_name="VGG-Face")
        self.target_size = self.model.input_shape
        self.person_image=None



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
    
    def image_callback(self, msg, from_path = False):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        is_good_quality = self.is_image_quality_acceptable(cv_image)
        print(is_good_quality, "is_good_qualityis_good_qualityis_good_qualityis_good_quality")
        # annotated_image, _, cropped_image = self.process_image(cv_image)
        if self.is_image_quality_acceptable(cv_image):
            self.annotated_image = self.process_image(cv_image)
            if self.annotated_image is not None:
                # cv2.imshow('Annotated Image', annotated_image)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                # Publish the annotated image
                self.person_image=self.annotated_image
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.annotated_image, encoding='bgr8'))
                # self.crop_pub.publish(self.bridge.cv2_to_imgmsg(cropped_image, encoding='bgr8'))
             
            else:
                rospy.logerr("No annotated image")
        else:
            rospy.logwarn("is_image_quality_acceptable is False")

    def calculate_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


    def check_nose_and_shoulder(self, landmarks):
        # Check if the nose keypoint exists
        nose_exists = landmarks[mp_pose.PoseLandmark.NOSE.value].visibility > 0

        # Check if the left shoulder keypoint exists
        shoulder_exists = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].visibility > 0

        return nose_exists, shoulder_exists
    
    def get_keypoint_coordinates(self, landmarks):
        # Get coordinates of nose and left shoulder keypoints
        nose_point = (landmarks[mp_pose.PoseLandmark.NOSE.value].x,
                      landmarks[mp_pose.PoseLandmark.NOSE.value].y)
        
        left_shoulder_point = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                               landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y)

        return nose_point, left_shoulder_point

    def check_confidence(self, img_landmarks, person_detected = False):
        confidence = np.mean([landmark.visibility for landmark in img_landmarks])
        rospy.loginfo("Confidence of the detected person: {}".format(confidence))
        if confidence > 0.40:
            person_detected = True
            rospy.loginfo("Person detected with the good confidence {}".format(confidence))
        else:
            person_detected = False
            rospy.logwarn("No person detected due to low confidence {}".format(confidence))
        
        return person_detected

    def process_image(self, image, max_attempts=3):
        threshold_distance = 0.2
        # Convert image to RGB
        self.image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print(self.image_rgb)

        # Detect poses
        with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            # Recolor image to RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False

            # Make pose detection
            results = pose.process(rgb_image)

            # Recolor back to BGR
            rgb_image.flags.writeable = True
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # Render pose landmarks on the image if poses are detected
            if results.pose_landmarks:
                
                landmarks = results.pose_landmarks.landmark
                # finding shoulder and nose 
                nose_exists, shoulder_exists = self.check_nose_and_shoulder(landmarks)
                
                if nose_exists:
                    nose_point, left_shoulder_point = self.get_keypoint_coordinates(landmarks)
                    
                    # Calculate distance between nose and left shoulder keypoints
                    distance = self.calculate_distance(nose_point, left_shoulder_point)
                    rospy.loginfo(" distance {}".format(distance))

                    # Calculate confidence of the detected person
                    detected = self.check_confidence(landmarks)
                    # Determine if the person is in front of the camera or far away
                    if distance > threshold_distance and detected:
                        # Draw annotations on the image
                        self.annotated_image = bgr_image.copy()
                        mp_drawing.draw_landmarks(
                        self.annotated_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                        mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                        mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                        )

                        # Draw poses on top of the annotated image
                        mp_drawing.draw_landmarks(
                            self.annotated_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2),
                            mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2)
                        )

                        rospy.loginfo("Person is near by, start capturing the detected person")
                        rospy.loginfo("Person detected {}".format(self.annotated_image))
                        return self.annotated_image
                        
                    else:
                        rospy.logwarn("distance is lesser than the threshold {}".format(distance))
                        rospy.logwarn("Person is far from the lucy's view")
                        rospy.logerr("Low confidence, no person found!!!!")

                        return None 

                else:
                    rospy.logerr("Person's nose is not detected")
                    return None 
            else:
                rospy.logerr("Pose not found, person couldn't be detetcted")
                return None

    def execute(self, userdata):
        
        rospy.loginfo("Initiating receptionist interaction...")

        rospy.sleep(2)  # Small delay before greeting

        # Initial greeting
        self.say_this("Hello, I am Lucy, here to welcome you. May I know your name?")
        rospy.sleep(2)
        name_response = self.listen_and_transcribe()
       
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
        if self.person_image:
            self.person_data["person_image"]=self.person_image
        else:
            self.say("Person Could not be detected")
            return False

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


