import rospy
import std_msgs
from .detection_service_proxy import DetectionServiceProxy
from mcr_perception_msgs.msg import PlaneList
from mdr_perception_libs import Constant


class ObjectDetector(object):
    def __init__(self, detection_service_proxy, event_out_topic):
        if not isinstance(detection_service_proxy, DetectionServiceProxy):
            raise ValueError('argument 1 is not a DetectionServiceProxy instance')

        self._detection_service_proxy = detection_service_proxy
        self._event_out_pub = rospy.Publisher(event_out_topic, std_msgs.msg.String, queue_size=1)
        self._plane_list = None
        pass

    def start_detect_objects(self):
        self._plane_list = self._detection_service_proxy.get_objects_and_planes()
        if not isinstance(self._plane_list, PlaneList):
            raise ValueError('get_objects_and_planes() did not return a PlaneList instance')
        self._event_out_pub.publish(Constant.E_SUCCESS)
        return

    @property
    def plane_list(self):
        return self._plane_list
