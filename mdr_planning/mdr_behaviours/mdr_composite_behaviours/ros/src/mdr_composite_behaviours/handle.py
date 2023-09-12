#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
# from mas_perception_msgs.msg import BoundingBox2D
# from mas_perception_libs import 
import numpy as np
from mas_perception_libs.utils import crop_cloud_to_xyz
from sensor_msgs.msg import PointCloud2
from mas_perception_libs._cpp_wrapper import BoundingBox2DWrapper
# from .bounding_box import BoundingBox2D

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from mas_perception_msgs.msg import BoundingBox as BoundingBoxMsg

from mas_perception_libs._cpp_wrapper import BoundingBoxWrapper, BoundingBox2DWrapper
# from .ros_message_serialization import to_cpp, from_cpp

try:
    from cStringIO import StringIO as IOClass   # Python 2.x
except ImportError:
    from io import BytesIO as IOClass           # Python 3.x


def to_cpp(msg):
    """
    Serialize ROS messages to string
    :param msg: ROS message to be serialized
    :rtype: str
    """
    buf = IOClass()
    msg.serialize(buf)
    return buf.getvalue()


def from_cpp(serial_msg, cls):
    """
    Deserialize strings to ROS messages
    :param serial_msg: memory view or bytes of serialized ROS message
    :type serial_msg: str
    :param cls: ROS message class
    :return: deserialized ROS message
    """
    msg = cls()
    if isinstance(serial_msg, memoryview):
        return msg.deserialize(serial_msg.tobytes())
    if isinstance(serial_msg, bytes):
        return msg.deserialize(serial_msg)
    raise TypeError("'serial_msg' has unexpected type: " + type(serial_msg))


class BoundingBox(object):
    def __init__(self, cloud, normal):
        if not isinstance(cloud, PointCloud2):
            rospy.ROSException('Argument 1 is not a sensor_msgs/PointCloud2')

        serial_cloud = to_cpp(cloud)
        self._bounding_box = BoundingBoxWrapper(serial_cloud, normal)

    def get_pose(self):
        serial_pose = self._bounding_box.get_pose()
        return from_cpp(serial_pose, PoseStamped)

    def get_ros_message(self):
        serial_message = self._bounding_box.get_ros_message()
        return from_cpp(serial_message, BoundingBoxMsg)


class BoundingBox2D(BoundingBox2DWrapper):
    """
    Python interface to the BoundingBox2D C++ struct. Allow the use of the same utility functions available in C++
    from Python.
    properties available for read/write access: label, x, y, width, height
    """
    def __init__(self, label="", color=(0, 0, 255), box_geometry=(0, 0, 0, 0)):
        """
        :param label: name of box when visualized
        :type label: str
        :param color: (r, g, b) RGB color of box, default color is blue
        :type color: tuple
        :param box_geometry: (x, y, width, height) bounding box's pixel coordinates and size
        :type box_geometry: tuple
        """
        super(BoundingBox2D, self).__init__(label, color, box_geometry)

    @property
    def box_geometry(self):
        # type: () -> tuple
        return self.x, self.y, self.width, self.height


# class BoundingBox2D(BoundingBox2DWrapper):
#     """
#     Python interface to the BoundingBox2D C++ struct. Allow the use of the same utility functions available in C++
#     from Python.
#     properties available for read/write access: label, x, y, width, height
#     """
#     def __init__(self, label="", color=(0, 0, 255), box_geometry=(0, 0, 0, 0)):
#         """
#         :param label: name of box when visualized
#         :type label: str
#         :param color: (r, g, b) RGB color of box, default color is blue
#         :type color: tuple
#         :param box_geometry: (x, y, width, height) bounding box's pixel coordinates and size
#         :type box_geometry: tuple
#         """
#         super(BoundingBox2D, self).__init__(label, color, box_geometry)

#     @property
#     def box_geometry(self):
#         # type: () -> tuple
#         return self.x, self.y, self.width, self.height

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_composite_behaviours/ros/models/erl_door.pt')#, force_reload=True)  # or yolov5n - yolov5x6, custom # Change 'yolov5s.pt' to your model path
model.eval()  # Set the model to evaluation mode



class ImageListener:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_listener', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.callback)
        self.cloud_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.cloud_callback)
        # Load the YOLOv5 model
       
        self.cloud=None 

    def cloud_callback(self,msg):
        self.cloud = msg
        
    def callback(self, data):
        try:
            # Convert the ROS image to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Inference
        results = model(cv_image)
        results.print()  # Print results to stdout

        # Extract bounding box details
        pred = results.pred[0]
        # for *xyxy, conf, cls in pred:
        #     x1, y1, x2, y2 = map(int, xyxy)
        for *xyxy, conf, cls in pred:
            x1, y1, x2, y2 = map(int, xyxy)
            
            w = x2 - x1
            h = y2 - y1
            x = x1 + w//2
            y = y1 + h//2
            label = model.names[int(cls)]
            conf_score = float(conf)
            print(f"Detected {label} with confidence {conf_score:.2f} at [{x1}, {y1}, {x2}, {y2}]")

            # detection = BoundingBox2D()  # (x, y, width, height)

            # detection.min_x=x1
            # detection.min_y=y1
            # detection.max_x=x2
            # detection.max_x=y2

            # box_geometry = (box_dict[ImageDetectionKey.X_MIN],
            #                 box_dict[ImageDetectionKey.Y_MIN],
            #                 box_dict[ImageDetectionKey.X_MAX] -
            #                 box_dict[ImageDetectionKey.X_MIN],
            #                 box_dict[ImageDetectionKey.Y_MAX] - box_dict[ImageDetectionKey.Y_MIN])

            # label = '{}: {:.2f}'.format(
            #     box_dict[ImageDetectionKey.CLASS], box_dict[ImageDetectionKey.CONF])

            # if color_dict is None:
            #     color = (0, 0, 255)     # default color: blue
            # else:
            #     color = color_dict[box_dict[ImageDetectionKey.CLASS]]

            # bounding_box = BoundingBox2D(label, color, box_geometry)

            detection = BoundingBox2D(label='handle', color=(0,0,255),box_geometry=(x, y, w, h))  # (x, y, width, height)
                        # crop cloud for coordinates and estimate pose

            print("from handle file: ", id(detection))
            cropped_coord = crop_cloud_to_xyz(self.cloud, detection)
            mean_pose = np.nanmean(np.reshape(cropped_coord, (-1, 3)), axis=0)
            print('estimated handle box pose (frame {0}): x={1:.3f}, y={2:.3f}, z={3:.3f}'
                .format(self.cloud.header.frame_id, mean_pose[0], mean_pose[1], mean_pose[2]))

           
            # Draw the bounding box on the image (optional)
            # cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(img, f"{label} {conf_score:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (255,0,0), 2, cv2.LINE_AA)

if __name__ == '__main__':
    il = ImageListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
