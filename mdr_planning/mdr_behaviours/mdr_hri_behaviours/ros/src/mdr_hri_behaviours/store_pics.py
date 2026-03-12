#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RGBCapture:
    def __init__(self):
        rospy.init_node('rgb_capture_node', anonymous=True)
        self.bridge = CvBridge()
        
        # --- CONFIGURATION ---
        # Change this to your camera's RGB topic (e.g., /usb_cam/image_raw)
        self.image_topic = "/hsrb/head_rgbd_sensor/rgb/image_raw" 
        self.save_path = "./rgb_dataset2"
        # ---------------------

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        self.cv_image = None
        self.count = 0

        # Subscriber for RGB only
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        rospy.loginfo("RGB Capture Node started.")
        rospy.loginfo("CONTROLS: [SPACE] to save frame | [ESC] to quit")
        self.run()

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def run(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # Show continuous live feed
                cv2.imshow("RGB Live View", self.cv_image)
            
            # Listen for keyboard input (1ms delay)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # Space bar to capture
                if self.cv_image is not None:
                    self.save_image()
                else:
                    rospy.logwarn("No image received yet...")
            
            elif key == 27:  # ESC key to exit
                rospy.loginfo("Shutting down...")
                break

        cv2.destroyAllWindows()

    def save_image(self):
        filename = os.path.join(self.save_path, f"rgb_{self.count:04d}.png")
        cv2.imwrite(filename, self.cv_image)
        rospy.loginfo(f"Successfully saved: {filename}")
        self.count += 1

if __name__ == '__main__':
    try:
        RGBCapture()
    except rospy.ROSInterruptException:
        pass
