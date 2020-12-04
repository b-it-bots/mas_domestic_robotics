import tf
import rospy
import numpy as np

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class SelectObjectForPicking(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'select_object_for_picking',
                                   save_sm_state=save_sm_state,
                                   output_keys=['selected_object'],
                                   outcomes=['succeeded'])
        self.tf_listener = tf.TransformListener()
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_object_for_picking')

    def execute(self, userdata):
        closest_obj_index = 0
        closest_obj_distance = float('inf')
        for index, obj in enumerate(userdata.detected_objects):
            while not rospy.is_shutdown():
                try:
                    (obj_position, obj_orientation) = self.tf_listener.lookupTransform('/base_link', 
                                                                                       obj.pose.header.frame_id, 
                                                                                       rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            distance_to_obj = np.linalg.norm(np.array(obj_position))

            if distance_to_obj < closest_obj_distance:
                closest_obj_index = index
                closest_obj_distance = distance_to_obj

        userdata.selected_object = userdata.objects[closest_obj_index]

        return 'succeeded'
