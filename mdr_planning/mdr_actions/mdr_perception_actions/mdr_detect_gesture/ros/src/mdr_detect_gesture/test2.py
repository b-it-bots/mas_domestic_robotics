import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from collections import deque
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        self.w=640
        self.h=480
        self.cv_image = np.zeros((480,640, 3),dtype=np.uint8)
        self.vid = cv2.VideoCapture(cv2.CAP_OPENNI)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.gesture_image_pub = rospy.Publisher("gesture_image", Image, queue_size=10)
        #self.image_pub3 = rospy.Publisher("image_topic1", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.callback)
        self.publish_img = []
        self.stopped = False
        self.pTime = time.time()
        self.d = deque(maxlen=30)
        self.fps=0
        time.sleep(3)
        # self.pc = rospy.Publisher("cropped_pc", PointCloud2, queue_size=1)

    def get_fps(self):
        self.cTime = time.time()
        fps1 = 1 / (self.cTime - self.pTime)
        self.d.append(fps1)
        self.fps = sum(self.d)/len(self.d)
        self.pTime = self.cTime
        return self.fps
    
    def img_publisher(self):
        Thread(target=self.publisher, args=()).start()
        return self
    
    def publisher(self):
        while not self.stopped:
            if not np.any(self.publish_img):
                print("yes")
                self.publish_img = self.cv_image.copy()
            try:
                #fps = self.get_fps()
                #cv2.putText(self.publish_img, f'FPS: {int(self.fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
                image_message = self.bridge.cv2_to_imgmsg(self.publish_img, encoding="passthrough")
                self.gesture_image_pub.publish(image_message)
                #print("gesture image published")
            except CvBridgeError as e:
                self.publish_img = np.zeros((480, 640, 3), dtype=np.uint8)
                #fps = self.get_fps()
                #cv2.putText(self.publish_img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
                image_message = self.bridge.cv2_to_imgmsg(self.publish_img, encoding="passthrough")
                self.gesture_image_pub.publish(image_message)
                print(e)
            time.sleep(0.02)
            #self.publish_img = []

    def callback(self,data):
        try:
            im = self.bridge.imgmsg_to_cv2(data)
            self.cv_image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            self.cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
            print(e)

    def get_frame(self):
        return self.cv_image
    
    def get_shape(self):
        return self.w, self.h
    
    def release_camera(self):
        self.stopped = True
        self.image_sub.unregister()

rospy.init_node('image_converter')
web = image_converter()

while 1:
    print(web.get_frame())