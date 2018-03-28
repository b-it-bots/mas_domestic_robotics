#!/usr/bin/python
import rospy
import smach
import std_msgs.msg
from mdr_perceive_plane_action.msg import PerceivePlaneResult, PerceivePlaneFeedback
from mcr_perception_msgs.msg import ObjectList


class SetupPlaneConfig(smach.State):
    def __init__(self, sleep_duration=1):
        smach.State.__init__(self, outcomes=['success', 'waiting', 'failure', 'timeout'],
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
                if self.event == "e_success":
                    return 'success'
                return 'failure'
            rate.sleep()
        return 'timeout'

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
        return 'waiting'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['perceive_plane_goal', 'recognized_objects'],
                             output_keys=['perceive_plane_feedback', 'perceive_plane_result'])
        self.result = result

    def execute(self, userdata):
        result = PerceivePlaneResult()
        result.success = self.result
        if userdata.recognized_objects is None:
            result.recognized_objects = ObjectList()
            result.recognized_objects.objects = []
        else:
            result.recognized_objects = userdata.recognized_objects
        userdata.perceive_plane_result = result
        return 'succeeded'
