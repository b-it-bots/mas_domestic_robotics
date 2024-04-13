# """
# Receptionist Challenge -2023
# Authors: Zain Ul Haq, Khawaja Saad, Ayusee Swain
# zainey4@gmail.com
# """
# #!/usr/bin/env python
# import rospy
# import random
# # import spacy
# # import speech_recognition as sr
# from polyglot.text import Text
# from mas_execution_manager.scenario_state_base import ScenarioStateBase
# from std_msgs.msg import String  # ROS standard string message
# from threading import Thread #import the threading module for running background task
# from deepface import DeepFace
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# from deepface.modules import verification
# from deepface.models.FacialRecognition import FacialRecognition
# from deepface.commons.logger import Logger
# import matplotlib.pyplot as plt




"""
Receptionist Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad, Ayusee Swain
zainey4@gmail.com
"""
#!/usr/bin/env python
import rospy
import random
# import spacy
# import speech_recognition as sr
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
class VerifyPerson():
    def __init__(self):
        self.topic_name = "/annotated_image"
        # self.topic_name = "/hsrb/head_rgbd_sensor/rgb/image_raw"
        self.model = DeepFace.build_model(model_name="VGG-Face")
        self.target_size = self.model.input_shape
        # self.database = dict ()      
        # Initialize the node
        rospy.init_node('image_subscriber_node', anonymous=True)
        # Subscribe to the image topic
        self.subscriber = rospy.Subscriber(self.topic_name,Image, self.image_callback)  

        # Create a CvBridge object for converting ROS images to OpenCV format
        self.bridge = CvBridge()        
        rospy.loginfo(f"Subscribed to {self.topic_name}")
        self.recognized_image = None
        self.person_data = {'guest_name':'', 'favorite_drink':'', 'person_image':None}
        self.guest_data = None
        self.image = None
        self.person = None
        self.verified_guest_name = None
        self.img_flag=False

#---------------------------------------------------------------------

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        # self.say(text)
        print(text)

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to an OpenCV image
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Image Window", self.image)

            self.img_flag=True
            # self.recognized_image = self.execute(image)
            # print(result)
            # cv2.imshow("Debug", cv_image)
            # cv2.waitKey(0)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def execute(self,userdata):
        print("startings")
        while(not self.img_flag):
            print('waiting for image...')
        print("checking....")
        faces = DeepFace.extract_faces(self.image, target_size=self.target_size)[0]["face"] 
        print("Order of the face matrix:", faces.shape)
        faces = np.expand_dims(faces, axis=0)
        embedding =  self.model.find_embeddings(faces)
        embedding = np.array(embedding)
        print("Checking the structure of each item in self.guest_data:")
        # for item in self.guest_data:
        #     print(type(item), item)

        self.guest_data = userdata
        for user_dic in self.guest_data:
            print(type(user_dic),user_dic)
            print('Checking...')
            # self.guest_data=self.guest_data[0]
            person=user_dic["person_image"]
            print("Order of the person matrix:",person.shape)
            person = np.expand_dims(person, axis=0)
            person =  self.model.find_embeddings(person)
            person = np.array(person)
            # reference= self.person_data [user_dic]        
            distance_vector = np.square(embedding-person)
            current_distance = np.sqrt(distance_vector.sum())
            threshold = verification.find_threshold(model_name="VGG-Face", distance_metric="euclidean")
            print(current_distance)

            if current_distance < threshold:
                self.verified_guest_name = user_dic['guest_name']
                self.verified_guest_drink = user_dic['favorite_drink']                    
                logger.info(
                    f"This person is {self.verified_guest_name}. {current_distance}"
                    f" is less than threshold {threshold}"
                )
            else:
                logger.info(
                    f"The amatch does not exist in the database. {current_distance}"
                    f" is greater than threshold {threshold}"
                )                    




        
userdata=np.load("sample_persons_record.npy",allow_pickle=True)
# print(userdata)

# name=userdata["guest_name"]
# drink=userdata["favorite_drink"]
# image=userdata["person_image"]
# print(type(userdata))
# print(name)
verifier= VerifyPerson()
verifier.execute(userdata)
    
