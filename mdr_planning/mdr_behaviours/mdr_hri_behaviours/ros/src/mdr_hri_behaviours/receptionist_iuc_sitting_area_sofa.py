"""
Receptionist Challenge -2024
Authors: Ayusee Swain
ayusee.1998@gmail.com
"""

import rospy
import speech_recognition as sr
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import String  # ROS standard message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
import random


class GuestIntroductionTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'guest_sitting_area_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['persons_record'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        self.person_data=None
        self.person_img_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw',Image, callback=self.callback)
        # Load the YOLOv8 model
        self.model = YOLO("yolov8n.pt")
        self.annotated_image = None
        self.static_image = None
        self.bridge = CvBridge()
        self.count = 0
        self.try_number = 3
        self.free_spaces = None
        self.location_to_sit = None


    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def get_img(self, image):
        self.static_image = image

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.annotated_image = cv_image

    def assign_occupancy_to_couch(self, occupancy_list, sitting_position):
        available_spaces = []
        for i, item in enumerate(occupancy_list):
            if item == "unoccupied":
                available_spaces.append(sitting_position[i])
        return available_spaces

    def execute(self, userdata):
        rospy.loginfo("Initiating guest introduction...")
        self.person_data=userdata.persons_record[-1]
        guest_name=self.person_data["guest_name"]
        favorite_drink=self.person_data["favorite_drink"]
        
        # loop for detecting couch
        while self.count < self.try_number:
            self.get_img(self.annotated_image)
            results = self.model.predict(self.static_image, classes=57, max_det=1)
            if len(results[0]) != 0:
                break
            self.count += 1
        # If cannot detect couch 3 times then return failed
        if len(results[0]) == 0:
            rospy.loginfo("Could not detect couch")
            return 'failed'
        
        couch_bboxes = []

        # Extract all couch bounding boxes
        for result in results:
            # if int(result.boxes.cls.numpy()) == couch_index:
            bbox = result.boxes.xyxy.numpy()
            for b in bbox:
                couch_bboxes.append(b)

        # Process each detected couch
        # orig_img_copy = self.static_image.copy()
        for couch_bbox in couch_bboxes:
            xmin, ymin, xmax, ymax = map(int, couch_bbox)
            width = xmax - xmin

            # Split the bounding box into three equal parts horizontally
            split_width = width / 3
            sections = [
                [xmin, ymin, xmin + split_width, ymax],
                [xmin + split_width, ymin, xmin + 2 * split_width, ymax],
                [xmin + 2 * split_width, ymin, xmax, ymax]
            ]

            # Detect persons in the image
            person_results = self.model.predict(self.static_image, classes=[0], max_det=3)  # Assuming index 0 is for 'person'
            # person_detection = person_results[0].plot()

            # Initialize person_bboxes array
            person_bboxes = []
            for result in person_results:
                if result.boxes is not None:
                    for box in result.boxes.xyxy.numpy():
                        person_bboxes.append(box)

            occupancy = ["unoccupied"] * 3  # Default to all sections being unoccupied
            sitting_location = ['left', 'center', 'right']

            # Check occupancy for each section if there are person detections
            if person_bboxes:
                for i, section in enumerate(sections):
                    for person_bbox in person_bboxes:
                        # Calculate the center point of the person's bounding box
                        person_center_x = (person_bbox[0] + person_bbox[2]) / 2
                        person_center_y = (person_bbox[1] + person_bbox[3]) / 2

                        # Check if the center point is within the section
                        if section[0] <= person_center_x <= section[2] and section[1] <= person_center_y <= section[3]:
                            occupancy[i] = "occupied"
            
            self.free_spaces = self.assign_occupancy_to_couch(occupancy, sitting_location)
            if len(self.free_spaces) == 0:
                self.location_to_sit = None
            else:
                self.location_to_sit = random.choice(self.free_spaces)


        if self.location_to_sit is None:
            introduction_phrase = f"{guest_name}, please be seated on the couch. It was a pleasure assisting you."
        else:
            introduction_phrase = f"{guest_name}, please be seated on the {self.location_to_sit} side of the couch. It was a pleasure assisting you."

        # guest_name = userdata.guest_name if 'guest_name' in userdata else "the guest"
        # favorite_drink = userdata.favorite_drink if 'favorite_drink' in userdata else "their favorite drink"

        introduction_phrase = f"{guest_name}, please be seated on the couch. It was a pleasure assisting you."

        #introduction_phrase = f"Hi Everyone, please meet {guest_name}. Their favorite drink is {favorite_drink}. It was a pleasure assisting you today. Have a wonderful time. Goodbye!"
        
        self.say_this(introduction_phrase)
        rospy.sleep(2)  # Give some time for people in the room to acknowledge the introduction

        rospy.loginfo("Guest introduced successfully.")
        return 'succeeded'
