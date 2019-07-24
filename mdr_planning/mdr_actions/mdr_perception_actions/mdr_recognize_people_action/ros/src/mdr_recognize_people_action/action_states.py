import yaml
import rospy
from rospkg import ResourceNotFound

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mas_perception_libs import KerasImageClassifier
from mas_tools.ros_utils import get_package_path
from mdr_recognize_people_action.msg import RecognizePeopleResult


class RecognizePeopleSM(ActionSMBase):
    _config_dict = None         # type: dict
    _timeout = None             # type: float
    _recognition_models = None  # type: dict

    def __init__(self, config_file, timeout=120.0,
                 max_recovery_attempts=1):
        super(RecognizePeopleSM, self).__init__('RecognizePeople', [], max_recovery_attempts)
        self._timeout = timeout
        self._recognition_models = {}
        with open(config_file) as infile:
            if yaml.__version__ < '5.1':
                self._config_dict = yaml.load(infile)
            else:
                self._config_dict = yaml.load(infile, Loader=yaml.FullLoader)

    def init(self):
        rospy.loginfo('[recognize_people] initiialising')
        # load detection models

        # load recognition models
        for recog_model_name, model_info in self._config_dict['recognition_models'].items():
            rospy.loginfo("[recognize_people] loading '{}' recognition model".format(recog_model_name))
            if model_info['type'] == 'keras':
                recog_model = RecognizePeopleSM._load_keras_recognition_model(model_info['configs'])
                if not recog_model:
                    rospy.logerr("loading recognition model '{}' of type '{}' failed"
                                 .format(recog_model_name, model_info['type']))
                    continue
                self._recognition_models[recog_model_name] = recog_model
            else:
                rospy.logerr("model type '{}' not supported (for recognizing '{}')"
                             .format(model_info['type'], recog_model_name))

        # load identity model

        # load age model

        return FTSMTransitions.INITIALISED

    def running(self):
        rospy.loginfo('[recognize_people] Recognizing people')
        self.result = self.set_result()
        return FTSMTransitions.DONE

    def set_result(self):
        result = RecognizePeopleResult()
        return result

    @staticmethod
    def _load_keras_recognition_model(model_configs):
        try:
            model_path = get_package_path(model_configs['model_package'], *model_configs['model_file'])
            class_file_path = get_package_path(model_configs['model_package'], *model_configs['class_file'])
            return KerasImageClassifier(model_path=model_path, class_file=class_file_path)
        except ResourceNotFound:
            rospy.logerr("package '{}' containing recognition model not found" .format(model_configs['model_package']))
        except Exception as e:
            rospy.logerr("error create ImageClassifier object: " + e.message)
        return None
