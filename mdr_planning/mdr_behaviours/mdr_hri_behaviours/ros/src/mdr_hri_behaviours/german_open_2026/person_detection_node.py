#!/usr/bin/env python3
"""
Receptionist ROS Node - 2026
Author: Khawaja Saad
saadbutt15@outlook.com
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO

# Importing your existing custom logic
from mdr_composite_behaviours.coordetector import t2d2t3d

class PersonDetectionNode:
    def __init__(self):
        rospy.init_node('person_detection_yolo26_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.td23D = t2d2t3d()
        
        # Load YOLOv26 model
        # 0 is the COCO index for 'person'
        self.model = YOLO('yolo11n.pt') 
        
        # Internal storage for sync
        self.cv_image = None
        self.cloud_data = None
        
        # Subscribers
        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.cloud_callback)
        
        # Publisher for the final object pose
        self.pose_pub = rospy.Publisher("/detected_person_pose", PoseStamped, queue_size=1)
        
        rospy.loginfo("YOLOv26 Person Detection Node Initialized.")

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            rospy.loginfo(f"Image captured!!!!!")
            self.process_detection()
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def cloud_callback(self, data):
        self.cloud_data = data

    def shrink_box(self, box, width_scale=0.6, height_scale=0.7):
        x1, y1 = box[0]
        x2, y2 = box[1]

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = (x2 - x1) * width_scale
        h = (y2 - y1) * height_scale

        nx1 = int(cx - w / 2.0)
        ny1 = int(cy - h / 2.0)
        nx2 = int(cx + w / 2.0)
        ny2 = int(cy + h / 2.0)

        return [[nx1, ny1], [nx2, ny2]]

    def process_detection(self):
        if self.cv_image is None or self.cloud_data is None:
            return

        # YOLOv26 Inference
        results = self.model.predict(self.cv_image, classes=[0], conf=0.6, verbose=False)
        
        detected = False
        box = None
        
        for result in results:
            if len(result.boxes) > 0:
                # YOLOv26 boxes are tensors; convert to numpy for your script
                b = result.boxes[0].xyxy[0].cpu().numpy() 
                box = [[int(b[0]), int(b[1])], [int(b[2]), int(b[3])]]
                detected = True
                rospy.loginfo("Person detected !!!!!!")
                break

        if detected:
            try:
                # Call your existing 3D coordinate script
                # Ensure self.td23D.get_3D_cords returns a list/tuple like [x, y, z]
                small_box = self.shrink_box(box, width_scale=0.55, height_scale=0.65)
                whole, obj_clus = self.td23D.get_box_voxel(small_box, self.cloud_data)
                obj_pose = self.td23D.get_3D_cords(obj_clus)

                # rospy.loginfo(f"obj_pose is {obj_pose}")
                
                # CORRECT WAY to fill a PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "head_rgbd_sensor_link" 
                
                # Assign values using DOT NOTATION, not brackets []
                pose_msg.pose.position.x = obj_pose.pose.position.x
                pose_msg.pose.position.y = obj_pose.pose.position.y
                pose_msg.pose.position.z = obj_pose.pose.position.z
                
                # Standard neutral orientation
                pose_msg.pose.orientation.x = obj_pose.pose.orientation.x
                pose_msg.pose.orientation.y = obj_pose.pose.orientation.y
                pose_msg.pose.orientation.z = obj_pose.pose.orientation.z
                pose_msg.pose.orientation.w = obj_pose.pose.orientation.w
                
                self.pose_pub.publish(pose_msg)
                
            except Exception as e:
                rospy.logerr(f"Error in coordinate transformation: {e}")
        else:
            # Optional: Log or publish an empty state if no one is found
            rospy.loginfo("No person detected")
            pass

if __name__ == '__main__':
    try:
        node = PersonDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass