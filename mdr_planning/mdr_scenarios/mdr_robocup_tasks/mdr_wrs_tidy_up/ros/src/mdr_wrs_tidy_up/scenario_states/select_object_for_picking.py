import tf
import rospy
import numpy as np

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class SelectObjectForPicking(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'select_object_for_picking',
                                   save_sm_state=save_sm_state,
                                   input_keys=['detected_objects'],
                                   output_keys=['selected_object'],
                                   outcomes=['succeeded'])
        self.tf_listener = tf.TransformListener()
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_object_for_picking')

    def execute(self, userdata):
        closest_obj_index = 0
        closest_obj_distance = float('inf')
        for index, obj in enumerate(userdata.detected_objects):
            try:
                pose_base_link = self.tf_listener.transformPose('base_link', obj.pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logerr('[%s] %s', self.state_name, str(exc))
                continue

            # we want to grasp the closest object along the xy-plane
            distance_to_obj = np.linalg.norm(np.array([pose_base_link.pose.position.x,
                                                       pose_base_link.pose.position.y]))

            if distance_to_obj < closest_obj_distance:
                closest_obj_index = index
                closest_obj_distance = distance_to_obj

        userdata.selected_object = userdata.detected_objects[closest_obj_index]
        return 'succeeded'
