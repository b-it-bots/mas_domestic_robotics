
#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class SofaDetector:
    def __init__(self):
        rospy.init_node('sofa_detector', anonymous=True)
        self.model = YOLO("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/sofa_model_yolo/best.pt")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert the ROS image message to a cv2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Run YOLO model prediction on the cv2 image
        result = self.model.predict(cv_image)

        # Process the result to print bounding boxes or any other processing
        if result is not None:
            for detection in result:
                if hasattr(detection, 'boxes'):
                    for box in detection.boxes:
                        bbox = np.array(box.xyxy).flatten()
                        cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 2)
                        # print(f'Box data is:\n{bbox}')
                        print(f'Debug: bbox shape={bbox.shape}, values={bbox}') 
                        
            #Display the image
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(0) 
        else:
            print("No detections were made.")

if __name__ == '__main__':
    try:
        sofa_detector = SofaDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
