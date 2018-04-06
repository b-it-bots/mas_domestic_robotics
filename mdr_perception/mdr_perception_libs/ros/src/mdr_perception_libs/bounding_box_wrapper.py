import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from mcr_perception_msgs.msg import BoundingBox as BoundingBoxMsg
from .ros_message_serialization import to_cpp, from_cpp

from mdr_perception_libs._cpp_wrapper import BoundingBoxWrapper


class BoundingBox(object):
    def __init__(self, cloud, normal):
        if not isinstance(cloud, PointCloud2):
            rospy.ROSException('Argument 1 is not a sensor_msgs/PointCloud2')
            pass

        serial_cloud = to_cpp(cloud)
        self._bounding_box = BoundingBoxWrapper(serial_cloud, normal)
        pass

    def get_pose(self):
        serial_pose = self._bounding_box.get_pose()
        return from_cpp(serial_pose, PoseStamped)

    def get_ros_message(self):
        serial_message = self._bounding_box.get_ros_message()
        return from_cpp(serial_message, BoundingBoxMsg)
