import cv2
import rospy
import numpy as np
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mcr_perception_msgs.msg import Person
from mdr_perception_msgs.msg import PersonInfo
from cv_bridge import CvBridge, CvBridgeError

class DescribePerson(ScenarioStateBase):

    COLOR_MAP_NAMES = ['red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'red']
    COLOR_MAP_HUES = [30, 90, 150, 210, 270, 330, 360]

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'describe_person',
                                   save_sm_state=save_sm_state,
                                   input_keys=['person_name'],
                                   outcomes=['succeeded'])
        self.bridge = CvBridge()
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'describe_person')

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        person_msg = self.kb_interface.get_obj_instance(userdata.person_name, Person)
        person_info_msg = self.kb_interface.get_obj_instance(userdata.person_name, PersonInfo)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(person_msg.rgb_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e.msg)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_mean = np.mean(hsv.reshape((-1,3)), axis=0)
        hue_value = hsv_mean[0] / 255 * 360  # convert to degrees
        person_info_msg.clothes_colour = self.get_color_name(hue_value)

        self.kb_interface.update_obj_instance(userdata.person_name, PersonInfo)

        rospy.loginfo('Person described successfully')
        return 'succeeded'

    def get_color_name(self, hue_val):
        for i in range(len(self.COLOR_MAP_NAMES)):
            color_hue = self.COLOR_MAP_HUES[i]
            if hue_val < color_hue:
                return self.COLOR_MAP_NAMES[i]
        raise ValueError('hue value invalid: ' + hue_val)
