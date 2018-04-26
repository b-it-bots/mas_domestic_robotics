from importlib import import_module
import rospy
from mdr_perception_libs.srv import ImageRecognition, ImageRecognitionRequest


class ImageRecognitionServiceProxy(object):
    def __init__(self, service_name, model_name,  preprocess_input_module=None):
        rospy.wait_for_service(service_name, timeout=5.0)
        try:
            self._recog_proxy = rospy.ServiceProxy(service_name, ImageRecognition)
        except rospy.ServiceException as e:
            rospy.logerr('failed to get proxy for service ' + e.message)
            raise

        self._model_name = model_name

        self._preprocess_input_func = None
        if preprocess_input_module:
            self._preprocess_input_func = getattr(import_module(preprocess_input_module), 'preprocess_input')

    def recognize_images(self, image_messages, done_callback_func=None):
        request = ImageRecognitionRequest()
        request.images = image_messages
        request.model_name = self._model_name
        response = self._recog_proxy(request)
        if done_callback_func is not None:
            done_callback_func()

        return response.indices, response.classes, response.probabilities
