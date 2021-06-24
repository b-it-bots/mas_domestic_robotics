import random

import tf
import rospy
import numpy as np

from std_msgs.msg import String

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class GraspingContext(object):
    CLEAN_UP = 'clean_up'
    GO_AND_GET_IT = 'go_and_get_it'

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

        self.grasping_context = kwargs.get('grasping_context', GraspingContext.CLEAN_UP)
        self.object_request_string = None
        self.requested_object_category = None
        self.allowed_requesting_people = ['person_right', 'person_left']

        self.__init_ros_components()

    def execute(self, userdata):
        if self.grasping_context == GraspingContext.CLEAN_UP:
            rospy.loginfo('[%s] Selecting closest object...', self.state_name)
            selected_object = self.get_closest_object(userdata.detected_objects)
        elif self.grasping_context == GraspingContext.GO_AND_GET_IT:
            requesting_person = None
            if self.object_request_string:
                self.requested_object_category = self.object_request_string.split(' ')[0]
                person_string = '_'.join(self.object_request_string.split(' ')[-2:])
                if person_string in self.allowed_requesting_people:
                    requesting_person = person_string
                else:
                    rospy.logwarn('[%s] Invalid person specification: %s! Picking one at random',
                                  self.state_name, person_string)
                    requesting_person = random.choice(self.allowed_requesting_people)

                rospy.loginfo('[%s] Selecting %s to deliver to %s', self.state_name,
                              self.requested_object_category, requesting_person)
                for obj in userdata.detected_objects:
                    if obj.category and obj.category[4:] == self.requested_object_category:
                        rospy.loginfo('[%s] Found requested object among detected objects!', self.state_name)
                        rospy.loginfo('[%s] Selecting requested object: %s', self.state_name, self.requested_object_category)
                        selected_object = obj
                        break
                else:
                    rospy.logwarn('[%s] Could not find requested object among detected objects!', self.state_name)
                    rospy.loginfo('[%s] Defaulting to selecting closest object...', self.state_name)
                    selected_object = self.get_closest_object(userdata.detected_objects)
            else:
                rospy.loginfo('[%s] Request not received; selecting closest object...', self.state_name)
                selected_object = self.get_closest_object(userdata.detected_objects)

                rospy.logwarn('[%s] Picking person at random', self.state_name)
                requesting_person = random.choice(self.allowed_requesting_people)

            userdata.destination_locations = [requesting_person]
        else:
            rospy.loginfo('[%s] Unknown grasping context %s; selecting closest object...',
                          self.grasping_context, self.state_name)
            selected_object = self.get_closest_object(userdata.detected_objects)

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

            distance_to_obj = 1e10
            # in the case of a go and get it task, we select the closest object along x
            if self.grasping_context == GraspingContext.GO_AND_GET_IT:
                distance_to_obj = np.linalg.norm(np.array([pose_base_link.pose.position.x]))
            # in general, we want to grasp the closest object along the xy-plane
            else:
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
