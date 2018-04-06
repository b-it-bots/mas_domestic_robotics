import rospy
from .detection_service_proxy import DetectionServiceProxy
from mcr_perception_msgs.msg import PlaneList


class ObjectDetector(object):
    def __init__(self, detection_service_proxy):
        if not isinstance(detection_service_proxy, DetectionServiceProxy):
            raise ValueError('argument 1 is not a DetectionServiceProxy instance')

        self._detection_service_proxy = detection_service_proxy
        pass

    def detect_objects(self):
        plane_list = self._detection_service_proxy.get_objects_and_planes()
        if not isinstance(plane_list, PlaneList):
            raise ValueError('get_objects_and_planes() did not return a PlaneList instance')
        return plane_list

