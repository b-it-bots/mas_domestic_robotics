#!/usr/bin/env python3
"""
ROS1 Node: Face Detection with MediaPipe
-----------------------------------------
Subscribes to a  , uses MediaPipe Face Detection to confirm
a face is present, then saves the image to disk and publishes metadata.

Robot deployment notes:
  - Tested with ROS Noetic (Python 3)
  - Works with any sensor_msgs/Image source (USB cam, RealSense, etc.)
  - Saves detected face images to ~/face_captures/ by default

Dependencies:
  pip install mediapipe opencv-python
  sudo apt install ros-noetic-cv-bridge ros-noetic-sensor-msgs
"""

import os
import rospy
import cv2
import mediapipe as mp
import numpy as np
from datetime import datetime
from typing import Optional
import json
import smach

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError


class FaceDetectionNode(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['save_image', 'face_not_found'],
        )
        rospy.init_node("face_detection_node", anonymous=False)

        # for smach
        self.image_saved = False
        self.face_not_found = False

        # ── Parameters (set via ROS param server or launch file) ──────────────
        self.image_topic     = rospy.get_param("~image_topic",     "/camera/image_raw")
        self.save_dir        = rospy.get_param("~save_dir",        os.path.expanduser("~/face_captures"))
        self.min_confidence  = rospy.get_param("~min_confidence",  0.7)   # 0.0–1.0
        self.model_selection = rospy.get_param("~model_selection", 0)     # 0=short range(<2m), 1=full range
        self.save_annotated  = rospy.get_param("~save_annotated",  True)  # draw bboxes on saved image
        self.cooldown_secs   = rospy.get_param("~cooldown_secs",   2.0)   # min seconds between saves
        self.publish_debug   = rospy.get_param("~publish_debug",   True)  # republish annotated image

        # ── Output directory ──────────────────────────────────────────────────
        os.makedirs(self.save_dir, exist_ok=True)
        rospy.loginfo(f"[FaceDetection] Saving images to: {self.save_dir}")

        # ── MediaPipe setup ───────────────────────────────────────────────────
        self.mp_face  = mp.solutions.face_detection
        self.mp_draw  = mp.solutions.drawing_utils
        self.detector = self.mp_face.FaceDetection(
            model_selection=self.model_selection,
            min_detection_confidence=self.min_confidence,
        )
        # Pose detection
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1,
            buff_size=2**24  # large buffer for high-res cameras
        )

        # Publishers
        self.face_detected_pub = rospy.Publisher(
            "~face_detected", Bool, queue_size=10
        )
        self.saved_path_pub = rospy.Publisher(
            "~saved_image_path", String, queue_size=10
        )
        if self.publish_debug:
            self.debug_image_pub = rospy.Publisher(
                "~debug_image", Image, queue_size=1
            )

        # ── State ─────────────────────────────────────────────────────────────
        self.last_save_time = rospy.Time(0)
        self.total_saved    = 0

        rospy.loginfo(
            f"[FaceDetection] Ready. Listening on '{self.image_topic}' "
            f"| confidence≥{self.min_confidence} | cooldown={self.cooldown_secs}s"
        )

    # ──────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[FaceDetection] CvBridge error: {e}")
            return

        # MediaPipe expects RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False  # perf hint
        results = self.detector.process(rgb_image)
        rgb_image.flags.writeable = True

        face_found = results.detections is not None and len(results.detections) > 0
        self.face_detected_pub.publish(Bool(data=face_found))

        if not face_found:
            rospy.loginfo_throttle(2, "No face detected → searching")
            self.face_not_found = True
            return

        annotated = cv_image.copy()

        if face_found:

            # ---- Select closest face (largest bounding box) ----
            h, w, _ = cv_image.shape
            largest_face = None
            largest_area = 0

            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                area = bbox.width * bbox.height

                if area > largest_area:
                    largest_area = area
                    largest_face = detection

            if largest_face is not None:

                if self.save_annotated:
                    self._draw_detections(annotated, [largest_face], cv_image.shape)

                # ---- Run Pose Detection ----
                pose_results = self.pose.process(rgb_image)

                hips_visible = False

                if pose_results.pose_landmarks:

                    lm = pose_results.pose_landmarks.landmark

                    left_hip = lm[self.mp_pose.PoseLandmark.LEFT_HIP]
                    right_hip = lm[self.mp_pose.PoseLandmark.RIGHT_HIP]

                    if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
                        hips_visible = True

                    # draw pose skeleton
                    self.mp_draw.draw_landmarks(
                        annotated,
                        pose_results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS
                    )

                # ---- Save only if hips AND face visible ----
                # if hips_visible:

                now = rospy.Time.now()

                if (now - self.last_save_time).to_sec() >= self.cooldown_secs:

                    save_img = cv_image

                    # save_img = annotated if self.save_annotated else cv_image
                    path = self._save_image(save_img, 1)

                    if path:
                        self.last_save_time = now
                        self.saved_path_pub.publish(String(data=path)) 


                        json1_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person1.json"
                        json2_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/german_open_2026/person_json/person2.json"

                        file_path = json2_path if os.path.isfile(json2_path) else json1_path

                        rospy.loginfo(f"[FaceDetection] Updating JSON at: {file_path}")

                        with open(file_path, "r") as f:
                            data = json.load(f)

                        # add image path
                        data["guest1"]["image_path"] = path

                        with open(file_path, "w") as f:
                            json.dump(data, f, indent=4)

                        # rospy.loginfo("[FaceDetection] Face + hips detected → image saved")
                        rospy.loginfo("[FaceDetection] Face → image saved")
                        self.image_saved = True

                else:
                    rospy.loginfo_throttle(
                        2,
                        "[FaceDetection] Face detected but hips not visible"
                    )


                    # move() # move the robot to see if hips are visible

        # if face_found:
        #     num_faces = len(results.detections)
        #     rospy.loginfo_throttle(2, f"[FaceDetection] {num_faces} face(s) detected")

        #     if self.save_annotated:
        #         self._draw_detections(annotated, results.detections, cv_image.shape)

        #     # Save with cooldown to avoid flooding disk
        #     now = rospy.Time.now()
        #     if (now - self.last_save_time).to_sec() >= self.cooldown_secs:
        #         save_img = annotated if self.save_annotated else cv_image
        #         path = self._save_image(save_img, num_faces)
        #         if path:
        #             self.last_save_time = now
        #             self.saved_path_pub.publish(String(data=path))
        else:
            rospy.logdebug("[FaceDetection] No face detected in frame")

        # Publish debug image
        if self.publish_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logwarn(f"[FaceDetection] Debug publish error: {e}")
        # cv2.imshow("MediaPipe Face Detection", annotated)
        # cv2.waitKey(1)



    def execute(self, userdata):
        rospy.loginfo("[FaceDetection] SMACH state started")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.image_saved:
                rospy.loginfo("[FaceDetection] Returning outcome: save_image")
                self.image_saved = False
                return 'save_image'

            if self.face_not_found:
                rospy.logerr("[FaceDetection] Returning outcome: error")
                return 'face_not_found'

            rate.sleep()

    # ──────────────────────────────────────────────────────────────────────────
    def _draw_detections(self, image: np.ndarray, detections, shape):
        """Draw bounding boxes and keypoints onto image (in-place)."""
        h, w, _ = shape
        for detection in detections:
            # Draw MediaPipe landmarks
            self.mp_draw.draw_detection(image, detection)

            # Overlay confidence score
            score = detection.score[0] if detection.score else 0.0
            bbox  = detection.location_data.relative_bounding_box
            x1 = max(0, int(bbox.xmin * w))
            y1 = max(0, int(bbox.ymin * h))
            label = f"Face {score:.2f}"
            cv2.putText(
                image, label, (x1, max(y1 - 8, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
            )

    # ──────────────────────────────────────────────────────────────────────────
    def _save_image(self, image: np.ndarray, num_faces: int) -> Optional[str]:
        """Save image to disk; return full path or None on failure."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # ms precision
        filename  = f"face_{num_faces}x_{timestamp}.jpg"
        filepath  = os.path.join(self.save_dir, filename)
        try:
            success = cv2.imwrite(filepath, image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if success:
                self.total_saved += 1
                rospy.loginfo(
                    f"[FaceDetection] Saved #{self.total_saved}: {filepath}"
                )
                return filepath
            else:
                rospy.logwarn(f"[FaceDetection] cv2.imwrite failed for {filepath}")
        except Exception as e:
            rospy.logerr(f"[FaceDetection] Save error: {e}")
        return None

    # ──────────────────────────────────────────────────────────────────────────
    def shutdown(self):
        rospy.loginfo(
            f"[FaceDetection] Shutting down. Total images saved: {self.total_saved}"
        )
        self.detector.close()
        cv2.destroyAllWindows()


# ──────────────────────────────────────────────────────────────────────────────
def main():
    node = FaceDetectionNode()
    rospy.on_shutdown(node.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
