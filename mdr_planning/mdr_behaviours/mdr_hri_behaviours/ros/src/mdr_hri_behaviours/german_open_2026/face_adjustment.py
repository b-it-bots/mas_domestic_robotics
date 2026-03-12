#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import moveit_commander
import ros_numpy

class SimpleFaceChecker:
    def __init__(self):
        #rospy.init_node('face_checker')
        
        # Initialize YOLO
        self.model = YOLO('yolov8n-pose.pt')
        
        # Initialize MoveIt for head control
        self.head = moveit_commander.MoveGroupCommander("head")
        
        # Basic parameters
        self.face_found = False
        self.backward_speed = 0.2
        self.current_distance = None
        self.points_data = None
        
        # Head tilt positions - start lower and gradually increase
        self.head_positions = [0.3, 0.4, 0.5]  # Adjusted tilt angles
        self.current_head_position = 0
        
        # Face position parameters
        self.ideal_top_offset = 0.15  # Ideal distance from top of frame (15%)
        
        # Initialize CV bridge and publishers
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        
        # Subscribe to RGB camera and point cloud
        self.rgb_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/rgb/image_raw',
            Image, self.image_callback
        )
        self.cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self.cloud_callback
        )

    def cloud_callback(self, msg):
        self.points_data = ros_numpy.numpify(msg)

    def get_distance_to_point(self, x, y):
        if self.points_data is None:
            return None
            
        try:
            point_xyz = self.points_data[int(y), int(x)]
            distance = np.sqrt(point_xyz['x']**2 + point_xyz['y']**2 + point_xyz['z']**2)
            return distance
        except:
            return None

    def check_face_position(self, keypoints, frame_height):
        """Check if face is well-positioned in frame"""
        face_points = keypoints.data[0].cpu().numpy()[:3]  # Get nose and eyes
        top_point = np.min(face_points[:, 1])  # Get highest point of face
        top_ratio = top_point / frame_height
        
        return 0.1 <= top_ratio <= 0.2  # Check if face is properly positioned

    def check_face(self, frame):
        """Check face visibility and position"""
        results = self.model(frame, conf=0.3)
        frame_height = frame.shape[0]
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            keypoints = results[0].keypoints[0] if results[0].keypoints else None
            if keypoints is not None:
                face_points = keypoints.data[0].cpu().numpy()[:3]
                confidences = face_points[:, 2]
                
                if np.all(confidences > 0.5):
                    # Check face position in frame
                    if self.check_face_position(keypoints, frame_height):
                        # Get distance measurement
                        nose_x, nose_y = face_points[0, :2]
                        distance = self.get_distance_to_point(nose_x, nose_y)
                        if distance is not None:
                            self.current_distance = distance
                        return True
        return False

    def move_back(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -self.backward_speed
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def try_next_head_position(self):
        """Try next head position if available"""
        if self.current_head_position < len(self.head_positions):
            angle = self.head_positions[self.current_head_position]
            rospy.loginfo(f"Trying head position {self.current_head_position + 1} (angle: {angle})")
            self.head.set_joint_value_target("head_tilt_joint", angle)
            self.head.go()
            self.current_head_position += 1
            rospy.sleep(1.0)
            return True
        return False

    def image_callback(self, msg):
        if self.face_found:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Check for face and draw result
            self.face_found = self.check_face(frame)
            
            # Simple visualization
            status = "Face Found!" if self.face_found else "Searching..."
            cv2.putText(frame, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1,
                       (0, 255, 0) if self.face_found else (0, 0, 255), 2)
            
            if self.current_distance is not None:
                distance_text = f"Distance: {self.current_distance:.2f}m"
                cv2.putText(frame, distance_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Draw ideal face position guide
            h = frame.shape[0]
            ideal_y = int(h * self.ideal_top_offset)
            cv2.line(frame, (0, ideal_y), (frame.shape[1], ideal_y), 
                    (255, 255, 0), 1)
            
            # cv2.imshow("Face Check", frame)
            # cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error processing frame: {str(e)}")

    def find_face(self):
        """Main sequence to find face"""
        rospy.loginfo("Starting face search...")
        
        while not rospy.is_shutdown() and not self.face_found:
            # Try current head position
            rospy.sleep(1.0)
            
            if self.face_found:
                break
                
            # Try next head position or move back
            if not self.try_next_head_position():
                rospy.loginfo("Moving back...")
                self.move_back()
                self.current_head_position = 0  # Reset head position counter
                rospy.sleep(1.0)
                
            if self.face_found:
                break
        
        if self.face_found:
            rospy.loginfo(f"Success! Face found at distance: {self.current_distance:.2f}m")
            return True
            
        rospy.loginfo("Face not found after all attempts")
        return False

    def shutdown_hook(self):
        """Clean shutdown"""
        self.head.set_joint_value_target("head_tilt_joint", 0.0)
        self.head.go()
        # cv2.destroyAllWindows()

def main():
    try:
        checker = SimpleFaceChecker()
        rospy.on_shutdown(checker.shutdown_hook)
        
        if checker.find_face():
            rospy.loginfo("Successfully found face!")
            if checker.current_distance is not None:
                rospy.loginfo(f"Final distance from person: {checker.current_distance:.2f} meters")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()