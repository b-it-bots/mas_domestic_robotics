import random

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
                                   output_keys=['selected_object',
                                                'destination_locations'],
                                   outcomes=['succeeded'])
        self.tf_listener = tf.TransformListener()
        self.requested_object_category_topic = kwargs.get('requested_object_category_topic ',
                                                          '/message')
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_object_for_picking')

        self.object_request_string = None
        self.requested_object_category = None
        self.requesting_person = None

        self.__init_ros_components()

    def execute(self, userdata):
        if self.object_request_string is None:
            rospy.loginfo('[%s] Selecting closest object...', self.state_name)
            selected_object = self.get_closest_object(userdata.detected_objects)
        else:
            self.requested_object_category = self.object_request_string.split(' ')[0]
            person_string = '_'.join(self.object_request_string.split(' ')[-2:])
            if person_string in ['person_right', 'person_left']:
                self.requesting_person = person_string
            else:
                rospy.logwarn('[%s] Invalid person specification! Picking one at random', self.state_name)
                self.requesting_person = random.choice(['person_right', 'person_left'])

            rospy.loginfo('[%s] Selecting %s to deliver to %s', self.state_name, self.requested_object_category, self.requesting_person)
            for obj in userdata.detected_objects:
                if obj.category == self.requested_object_category:
                    rospy.loginfo('[%s] Found requested object among detected objects!', self.state_name)
                    rospy.loginfo('[%s] Selecting requested object: %s', self.state_name, self.requested_object_category)
                    selected_object = obj
                    break
            else:
                rospy.logwarn('[%s] Could not find requested object among detected objects!', self.state_name)
                rospy.loginfo('[%s] Defaulting to selecting closest object...', self.state_name)
                selected_object = self.get_closest_object(userdata.detected_objects)

            userdata.destination_locations = [self.requesting_person]

        rospy.loginfo('[%s] Selected object at position\n    (%f, %f, %f) \n with size\n    (%f, %f, %f) \n with label\n    "%s"',
                      self.state_name, selected_object.pose.pose.position.x,
                      selected_object.pose.pose.position.y,
                      selected_object.pose.pose.position.z,
                      selected_object.dimensions.vector.x,
                      selected_object.dimensions.vector.y,
                      selected_object.dimensions.vector.z,
                      selected_object.category)
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
        rospy.loginfo('[%s] Received object request: %s', self.state_name, msg.data)
        self.object_request_string = msg.data

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
