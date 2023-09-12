import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from mas_perception_msgs.msg import BoundingBox as BoundingBoxMsg

from mas_perception_libs._cpp_wrapper import BoundingBoxWrapper, BoundingBox2DWrapper
from .ros_message_serialization import to_cpp, from_cpp


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
