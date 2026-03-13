#!/usr/bin/env python3
"""
SaveFace state - Detect and save face image using MediaPipe.

Captures face from camera and saves to disk for later recognition.
"""

import os
import rospy
import smach
import cv2
import numpy as np
from datetime import datetime
from geometry_msgs.msg import Twist
import moveit_commander
import json

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    rospy.logwarn('[SaveFace] MediaPipe not available')

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError


class SaveFace(smach.State):
    """
    Detect a face in camera image and save it.
    
    Params:
        guest_number: 1 or 2 (for filename)
        image_topic: camera topic
        save_dir: directory to save face images
        min_confidence: face detection confidence threshold
        timeout: max time to wait for face detection
        retries: max retries before failing
    
    Outcomes:
        succeeded               - face saved successfully
        failed                  - temporary failure (retry)
        failed_after_retrying   - max retries exhausted
    
    Output keys:
        face_image_path - path to saved face image
    """

    def __init__(self,
                 guest_number=1,
                 image_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                 save_dir='/tmp/hri_faces',
                 min_confidence=0.6,
                 timeout=10.0,
                 retries=3):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['face_image_path']
        )
        self.guest_number = guest_number
        self.image_topic = image_topic
        self.save_dir = save_dir
        self.min_confidence = min_confidence
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0
        self.head = moveit_commander.MoveGroupCommander("head")

        # Basic parameters
        self.face_found = False
        self.backward_speed = 0.2
        
        # Create output directory
        os.makedirs(self.save_dir, exist_ok=True)

        # Head tilt positions - start lower and gradually increase
        self.head_positions = [0.3, 0.4, 0.5]  # Adjusted tilt angles
        self.current_head_position = 0
        
        # Face position parameters
        self.ideal_top_offset = 0.15  # Ideal distance from top of frame (15%)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # MediaPipe face detection
        if MEDIAPIPE_AVAILABLE:
            self.mp_face = mp.solutions.face_detection
            self.mp_draw = mp.solutions.drawing_utils
            self.detector = self.mp_face.FaceDetection(
                model_selection=0,  # 0 = short range (<2m)
                min_detection_confidence=self.min_confidence
            )
        else:
            self.detector = None
        
        # Publisher for TTS
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)

    def _say(self, text):
        """Publish text to /say topic."""
        self.say_pub.publish(String(data=text))
        num_words = len(text.split())
        rospy.sleep(max(0.5, num_words * 0.4))

    def _image_callback(self, msg):
        """Store latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received = True
        except CvBridgeError as e:
            rospy.logwarn('[SaveFace] CV bridge error: %s', e)

    def execute(self, userdata):
        self._say ("Please look at my camera, i would like to take a photo of you")
        if not MEDIAPIPE_AVAILABLE:
            rospy.logerr('[SaveFace] MediaPipe not installed')
            return 'failed_after_retrying'
        
        rospy.loginfo('[SaveFace] Waiting for face (guest %d)...', self.guest_number)
        
        # Subscribe to camera
        self.image_received = False
        self.latest_image = None
        image_sub = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
        )
        
        try:
            start_time = rospy.Time.now()
            face_detected = False
            saved_path = None
            
            while not rospy.is_shutdown():
                # Check timeout
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > self.timeout:
                    rospy.logwarn('[SaveFace] Timeout waiting for face')
                    break
                
                if not self.image_received or self.latest_image is None:
                    rospy.sleep(0.1)
                    continue
                
                # Convert to RGB for MediaPipe
                rgb_image = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB)
                results = self.detector.process(rgb_image)
                
                if results.detections:
                    for detection in results.detections:
                        confidence = detection.score[0]
                        rospy.loginfo('[SaveFace] Face detected (%.1f%% confidence)', 
                                      confidence * 100)
                        
                        if confidence >= self.min_confidence:
                            # Draw bounding box
                            annotated = self.latest_image.copy()
                            bbox = detection.location_data.relative_bounding_box
                            h, w, _ = annotated.shape
                            x = int(bbox.xmin * w)
                            y = int(bbox.ymin * h)
                            bw = int(bbox.width * w)
                            bh = int(bbox.height * h)
                            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), 
                                          (0, 255, 0), 2)
                            
                            # Save image
                            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                            filename = f'guest{self.guest_number}_{timestamp}.jpg'
                            saved_path = os.path.join(self.save_dir, filename)
                            cv2.imwrite(saved_path, annotated)
                
                if face_detected:
                    break
                
                rospy.sleep(0.1)
            
            if face_detected and saved_path:
                userdata.face_image_path = saved_path
                self.retry_count = 0
                return 'succeeded'
            else:
                return self._retry()
                
        finally:
            image_sub.unregister()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[SaveFace] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'
