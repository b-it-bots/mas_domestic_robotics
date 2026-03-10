#!/usr/bin/env python3
"""
PersonDetection smach state.

Waits for RGB image and point cloud data, runs person detection using YOLO,
computes the 3D pose of the detected person, and writes the result to userdata.

Outcomes
--------
detected     – person detected and pose computed successfully
notDetected  – image/cloud were available, but no valid person pose was found
timeout      – timed out waiting for messages or successful detection
"""

import rospy
import smach
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from ultralytics import YOLO

from mdr_composite_behaviours.coordetector import t2d2t3d


class PersonDetection(smach.State):
    def __init__(self,
                 image_topic="/hsrb/head_rgbd_sensor/rgb/image_raw",
                 cloud_topic="/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
                 model_path="yolo11n.pt",
                 conf_thresh=0.6,
                 wait_timeout=10.0,
                 poll_rate=10.0):
        smach.State.__init__(
            self,
            outcomes=["detected", "notDetected", "timeout"],
            output_keys=["obj_pose"] #This is going to be msgtype PoseStamped
        )

        self.image_topic = image_topic
        self.cloud_topic = cloud_topic
        self.model_path = model_path
        self.conf_thresh = conf_thresh
        self.wait_timeout = wait_timeout
        self.poll_rate = poll_rate

        self.bridge = CvBridge()
        self.td23D = t2d2t3d()
        self.model = YOLO(self.model_path)

        self.cv_image = None
        self.cloud_data = None
        self._image_received = False
        self._cloud_received = False

        self._image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        self._cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1
        )

        rospy.loginfo('[PersonDetection] Subscribed to "%s" and "%s"',
                      self.image_topic, self.cloud_topic)

    # ------------------------------------------------------------------ #

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self._image_received = True
        except CvBridgeError as e:
            rospy.logerr("[PersonDetection] CvBridge Error: %s", e)

    def cloud_callback(self, data):
        self.cloud_data = data
        self._cloud_received = True

    # ------------------------------------------------------------------ #

    def shrink_box(self, box, width_scale=0.55, height_scale=0.65):
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

    # ------------------------------------------------------------------ #

    def process_detection(self):
        if self.cv_image is None or self.cloud_data is None:
            return None

        results = self.model.predict(
            self.cv_image,
            classes=[0],
            conf=self.conf_thresh,
            verbose=False
        )

        detected = False
        box = None

        for result in results:
            if len(result.boxes) > 0:
                b = result.boxes[0].xyxy[0].cpu().numpy()
                box = [[int(b[0]), int(b[1])], [int(b[2]), int(b[3])]]
                detected = True
                rospy.loginfo("[PersonDetection] Person detected.")
                break

        if not detected:
            return False

        try:
            small_box = self.shrink_box(box, width_scale=0.55, height_scale=0.65)
            whole, obj_clus = self.td23D.get_box_voxel(small_box, self.cloud_data)
            obj_pose = self.td23D.get_3D_cords(obj_clus)

            if obj_pose is None:
                rospy.logwarn("[PersonDetection] obj_pose is None.")
                return False

            return obj_pose

        except Exception as e:
            rospy.logerr("[PersonDetection] Error in coordinate transformation: %s", e)
            return False

    # ------------------------------------------------------------------ #

    def execute(self, userdata):
        rospy.loginfo('[PersonDetection] Waiting up to %.1fs for image/cloud and valid detection...',
                      self.wait_timeout)

        self.cv_image = None
        self.cloud_data = None
        self._image_received = False
        self._cloud_received = False
        userdata.obj_pose = None

        rate = rospy.Rate(self.poll_rate)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()

            if self.wait_timeout >= 0 and elapsed >= self.wait_timeout:
                if not self._image_received or not self._cloud_received:
                    rospy.logwarn("[PersonDetection] Timeout: missing input messages.")
                    return "timeout"

                rospy.loginfo("[PersonDetection] Timeout: no valid person detected.")
                return "timeout"

            if not self._image_received or not self._cloud_received:
                rospy.loginfo_throttle(
                    2.0,
                    '[PersonDetection] Waiting for topics: image=%s cloud=%s',
                    self._image_received, self._cloud_received
                )
                rate.sleep()
                continue

            result = self.process_detection()

            if result is None:
                rate.sleep()
                continue

            if result is False:
                rospy.loginfo("[PersonDetection] No person detected in current data.")
                return "notDetected"

            userdata.obj_pose = result
            rospy.loginfo("[PersonDetection] Person pose computed successfully.")
            return "detected"

        return "timeout"