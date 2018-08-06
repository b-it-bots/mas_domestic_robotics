from abc import ABCMeta, abstractmethod
import rospy
from std_srvs.srv import Empty, EmptyRequest
from mcr_perception_msgs.msg import PlaneList, Plane

EMPTY_DETECTION_SERVICE = '/test_detection_service/execute'


class DetectionServiceProxy(object):
    __metaclass__ = ABCMeta

    def __init__(self, service_name, service_class):
        self._service_name = service_name
        try:
            rospy.loginfo('waiting for segmentation service "{0}" (type {1})'
                          .format(service_name, service_class.__name__))
            rospy.wait_for_service(service_name, timeout=5.0)
            self._detection_client = rospy.ServiceProxy(service_name, service_class)
        except rospy.ROSException:
            rospy.logerr('"{0}" service is not available'.format(service_name))
            raise

        self._detection_req = self._get_segmentation_req()

    def get_objects_and_planes(self):
        res = self._detection_client(self._detection_req)

        plane_list = self._get_objects_and_planes_from_response(res)
        if not isinstance(plane_list, PlaneList):
            raise ValueError('"_get_objects_and_planes_from_response()" did not return a "PlaneList" instance')

        #for i in range(len(plane_list.planes)):
        #    rospy.loginfo('plane [{0}] has[{1}] object(s)'.format(i, len(plane_list.planes[i].object_list.objects)))

        return plane_list

    @abstractmethod
    def _get_segmentation_req(self):
        pass

    @abstractmethod
    def _get_objects_and_planes_from_response(self, res):
        pass


class DetectionServiceProxyTest(DetectionServiceProxy):
    def __init__(self):
        super(DetectionServiceProxyTest, self).__init__(EMPTY_DETECTION_SERVICE, Empty)

    def _get_segmentation_req(self):
        return EmptyRequest()

    def _get_objects_and_planes_from_response(self, res):
        plane_list = PlaneList()
        plane_list.planes.append(Plane())
        return plane_list
