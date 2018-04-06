#!/usr/bin/python
import rospy
import smach
import std_msgs.msg
from mdr_perceive_plane_action.msg import PerceivePlaneResult, PerceivePlaneFeedback
from mcr_perception_msgs.msg import PlaneList
from mdr_object_recognition import ObjectDetector
from mdr_perception_libs import Constant


class DetectObjects(smach.State):
    def __init__(self, detection_service_proxy, detection_event_topic, timeout_duration=1):
        smach.State.__init__(self, outcomes=[Constant.SUCCESS, Constant.FAILURE, Constant.TIMEOUT],
                             output_keys=['detected_planes'])
        self._detector = ObjectDetector(detection_service_proxy, detection_event_topic)
        self._timeout_duration = timeout_duration
        self._detecting_done = False
        self._event_out_sub = rospy.Subscriber(detection_event_topic, std_msgs.msg.String, self._event_cb)
        return

    def _event_cb(self, event):
        self._detecting_done = True
        return

    def execute(self, ud):
        self._detecting_done = False
        ud.detected_planes = None

        self._detector.start_detect_objects()

        timeout = rospy.Duration.from_sec(self._timeout_duration)
        rate = rospy.Rate(10)   # 10Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < timeout:
            if self._detecting_done:
                if self._detector.plane_list is None:
                    return Constant.FAILURE

                ud.detected_planes = self._detector.plane_list
                return Constant.SUCCESS
            rate.sleep()
        return Constant.TIMEOUT


class RecognizeObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Constant.SUCCESS, Constant.FAILURE],
                             input_keys=['detected_planes'], output_keys=['recognized_planes'])
        pass

    def execute(self, ud):
        if 'detected_planes' not in ud:
            ud.recognized_planes = None
            return Constant.FAILURE

        ud.recognized_planes = ud.detected_planes
        return Constant.SUCCESS


class SetupPlaneConfig(smach.State):
    def __init__(self, sleep_duration=1):
        smach.State.__init__(self, outcomes=[Constant.SUCCESS, Constant.WAITING, Constant.FAILURE, Constant.TIMEOUT],
                             input_keys=['perceive_plane_goal'],
                             output_keys=['perceive_plane_feedback'])
        self.sleep_duration = sleep_duration

        self.config_name_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/configuration_name",
                                               std_msgs.msg.String, queue_size=1)
        self.event_in_pub = rospy.Publisher("/mcr_common/dynamic_reconfigure_client/event_in",
                                            std_msgs.msg.String, queue_size=1)
        self.event_out_sub = rospy.Subscriber("/mcr_common/dynamic_reconfigure_client/event_out",
                                              std_msgs.msg.String, self.event_cb)
        self.event = None

    def event_cb(self, msg):
        self.event = msg.data

    def configure_plane(self, config_name):
        """
        :param config_name: configurations name listed in
                            config/perceive_plane_configurations.yaml
        """
        self.event = None
        self.config_name_pub.publish(config_name)
        self.event_in_pub.publish("e_start")

        timeout = rospy.Duration.from_sec(1.0)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < timeout:
            if self.event:
                if self.event == Constant.E_SUCCESS:
                    return Constant.SUCCESS
                return Constant.FAILURE
            rate.sleep()
        return Constant.TIMEOUT

    def execute(self, userdata):
        feedback = PerceivePlaneFeedback()
        feedback.current_state = 'setup_plane_config'
        if 'perceive_plane_goal' in userdata:
            feedback.message = '[perceive_plane] received plane config goal ' +\
                               userdata.perceive_plane_goal.plane_config
            userdata.perceive_plane_feedback = feedback
            return self.configure_plane(userdata.perceive_plane_goal.plane_config)

        feedback.message = '[perceive_plane] waiting for plane config goal'
        userdata.perceive_plane_feedback = feedback
        rospy.sleep(self.sleep_duration)
        return Constant.WAITING


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=[Constant.SUCCESS],
                             input_keys=['perceive_plane_goal', 'recognized_planes'],
                             output_keys=['perceive_plane_feedback', 'perceive_plane_result'])
        self.result = result

    def execute(self, userdata):
        result = PerceivePlaneResult()
        result.success = self.result
        if userdata.recognized_planes is None:
            result.recognized_planes = PlaneList()
            result.recognized_planes.planes = []
        else:
            result.recognized_planes = userdata.recognized_planes
        userdata.perceive_plane_result = result
        return Constant.SUCCESS
