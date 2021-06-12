import tf
import rospy
import numpy as np

from std_msgs.msg import String

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class SelectObjectForPicking(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'select_object_for_picking',
                                   save_sm_state=save_sm_state,
                                   input_keys=['detected_objects'],
                                   output_keys=['selected_object'],
                                   outcomes=['succeeded'])
        self.tf_listener = tf.TransformListener()
        self.requested_object_category_topic = kwargs.get('requested_object_category_topic ',
                                                  '/message')
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_object_for_picking')

        self.requested_object_category = None

        self.__init_ros_components()

    def execute(self, userdata):
        if self.requested_object_category is None:
            rospy.loginfo('[%s] Selecting closest object...', self.state_name)
            selected_object = self.get_closest_object(userdata.detected_objects)
        else:
            for obj in userdata.detected_objects:
                if obj.category == self.requested_object_category:
                    rospy.loginfo('[%s] Found requested object among detected objects!', self.state_name)
                    rospy.loginfo('[%s] Selecting requested object: %s', self.state_name, self.requested_object_category)
                    selected_object = obj
                    break
            else:
                rospy.warn('[%s] Could not find requested object among detected objects!', self.state_name)
                rospy.loginfo('[%s] Defaulting to selecting closest object...', self.state_name)
                selected_object = self.get_closest_object(userdata.detected_objects)

        rospy.loginfo('[%s] Selected object at position\n    (%f, %f, %f) \n with size\n    (%f, %f, %f)',
                      self.state_name, selected_object.pose.pose.position.x,
                      selected_object.pose.pose.position.y,
                      selected_object.pose.pose.position.z,
                      selected_object.dimensions.vector.x,
                      selected_object.dimensions.vector.y,
                      selected_object.dimensions.vector.z)
        userdata.selected_object = selected_object
        return 'succeeded'

    def get_closest_object(self, detected_objects):
        closest_obj_index = 0
        closest_obj_distance = float('inf')
        for index, obj in enumerate(detected_objects):
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

        return detected_objects[closest_obj_index]

    def requested_object_category_callback(self, msg):
        rospy.loginfo('[%s] Received requested object category: %s', self.state_name, msg.data)
        self.requested_object_category = msg.data

    def __init_ros_components(self):
        '''Initialises:
        * a subscriber for the topic on which the category of a requested object is published
        '''
        rospy.loginfo('[%s] Creating subscriber for topic %s',
                      self.state_name, self.requested_object_category_topic)
        self.obstacle_sub = rospy.Subscriber(self.requested_object_category_topic,
                                             String, self.requested_object_category_callback)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Subscriber for topic %s created',
                      self.state_name, self.requested_object_category_topic)
