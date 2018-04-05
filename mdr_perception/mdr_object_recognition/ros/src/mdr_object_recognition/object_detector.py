import rospy
from sensor_msgs.msg import Image
import std_msgs
from .detection_service_proxy import DetectionServiceProxy


class ObjectDetector(object):
    def __init__(self, detection_service_proxy):
        if not isinstance(detection_service_proxy, DetectionServiceProxy):
            raise ValueError('argument 1 is not a DetectionServiceProxy instance')

        self._detection_service_proxy = detection_service_proxy
        self._event_in_sub = rospy.Subscriber('~event_in', std_msgs.msg.String, self._event_in_cb)
        self._img_pub = rospy.Publisher('~first_object', Image, queue_size=1)
        pass


    def _event_in_cb(self, event):
        object_list, plane_list = self._detection_service_proxy.get_objects_and_planes()
        if len(object_list.objects) == 0:
            return

        self._img_pub.publish(object_list.objects[0].rgb_image)
        pass


