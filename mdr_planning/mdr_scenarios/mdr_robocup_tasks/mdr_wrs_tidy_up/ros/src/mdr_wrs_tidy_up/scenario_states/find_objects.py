import numpy as np

import rospy
import tf
import actionlib
from std_msgs.msg import Bool

from mas_perception_msgs.msg import ObjectList, DetectObjectsAction, DetectObjectsActionGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class FindObjects(ScenarioStateBase):
    tf_listener = None
    object_detection_server_name = None
    cloud_obstacle_detection_topic = None
    cloud_obstacle_cache_reset_topic = None
    object_detection_client = None
    obstacle_sub = None
    detected_cloud_objects = None
    obstacle_cache_reset_pub = None
    last_cloud_object_detection_time = None
    retry_count = 0

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'find_objects',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'no_objects',
                                             'failed', 'failed_after_retrying'],
                                   input_keys=['environment_objects',
                                               'floor_objects_cleared',
                                               'table_objects_cleared',
                                               'object_location',
                                               'destination_locations'],
                                   output_keys=['detected_objects',
                                                'floor_objects_cleared',
                                                'table_objects_cleared'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'find_objects')
        self.object_detection_server_name = kwargs.get('object_detection_server_name',
                                                       '/mas_perception/detect_objects')
        self.cloud_obstacle_detection_topic = kwargs.get('cloud_obstacle_detection_topic',
                                                         '/mas_perception/cloud_obstacle_detection/obstacle_objects')
        self.cloud_obstacle_cache_reset_topic = kwargs.get('cloud_obstacle_cache_reset_topic',
                                                           '/mas_perception/cloud_obstacle_detection/reset_cache')
        self.max_allowed_obj_height_cm = kwargs.get('max_allowed_obj_height_cm', 0.15)
        self.min_allowed_dist_to_leg = kwargs.get('min_allowed_dist_to_leg', 0.15)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.__init_ros_components()

    def execute(self, userdata):
        last_msg_time = self.last_cloud_object_detection_time

        rospy.loginfo('[%s] Resetting cloud obstacle cache and waiting a bit', self.state_name)
        self.obstacle_cache_reset_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        rospy.loginfo('[%s] Waiting for cloud obstacle detection', self.state_name)
        while last_msg_time == self.last_cloud_object_detection_time:
            rospy.sleep(0.05)

        rospy.loginfo('[%s] Detected %d objects', self.state_name, len(self.detected_cloud_objects))

        # workaround for large objects (such as the pitcher) sticking to the gripper in Gazebo
        filtered_objects = self.filter_objects_by_height(self.detected_cloud_objects)
        filtered_objects = self.filter_objects_by_dist_to_table_legs(filtered_objects,
                                                                     userdata.environment_objects)
        userdata.detected_objects = filtered_objects

        # if no objects are seen in the current view, we register the location as "cleared"
        if not filtered_objects:
            current_location = userdata.destination_locations[0]
            userdata.floor_objects_cleared[current_location] = True
            return 'no_objects'

        return 'succeeded'

    def store_detected_objects(self, object_msg):
        '''Saves the objects received in the input message and stores the
        current time as the latest object detection time.

        Keyword arguments:
        object_msg: mas_perception_msgs.msg.ObjectList -- list of detected objects
        '''
        self.detected_cloud_objects = object_msg.objects
        self.last_cloud_object_detection_time = rospy.Time.now().to_sec()

    def filter_objects_by_height(self, objects):
        rospy.loginfo('[%s] Filtering detected objects using height threshold %f',
                      self.state_name, self.max_allowed_obj_height_cm)
        filtered_objects = [obj for obj in objects
                            if obj.dimensions.vector.z <= self.max_allowed_obj_height_cm]
        rospy.loginfo('[%s] Keeping %d objects', self.state_name, len(filtered_objects))
        return filtered_objects

    def filter_objects_by_dist_to_table_legs(self, objects, environment_objects):
        rospy.loginfo('[%s] Filtering detected objects using distance to table leg threshold %f',
                      self.state_name, self.min_allowed_dist_to_leg)

        leg_positions = []
        for env_obj_name, env_obj in environment_objects.items():
            if env_obj_name.find('leg') != -1:
                leg_positions.append(np.array([env_obj.pose.pose.position.x,
                                               env_obj.pose.pose.position.y]))
        leg_positions = np.array(leg_positions)

        filtered_objects = []
        for obj in objects:
            distances_to_legs = []
            obj_pose_in_map = self.tf_listener.transformPose('map', obj.pose)
            obj_xy_position = np.array([obj_pose_in_map.pose.position.x,
                                        obj_pose_in_map.pose.position.y])
            distances_to_legs = np.linalg.norm(leg_positions - obj_xy_position, axis=1)
            if np.min(distances_to_legs) > self.min_allowed_dist_to_leg:
                filtered_objects.append(obj)

        rospy.loginfo('[%s] Keeping %d objects', self.state_name, len(filtered_objects))
        return filtered_objects

    def __init_ros_components(self):
        '''Initialises:
        * an object detection action client
        * a subscriber for objects detected by a cloud-based processing component
        * a publisher for resetting the object cache of the cloud-based processing component
        '''
        # we create an action client for an image-based object detector
        rospy.loginfo('[%s] Initialising client for action %s',
                      self.state_name, self.object_detection_server_name)
        self.object_detection_client = actionlib.SimpleActionClient(self.object_detection_server_name,
                                                                    DetectObjectsAction)
        rospy.loginfo('[%s] Client for action %s initialised',
                      self.state_name, self.object_detection_server_name)

        # we create a subscriber for objects detected by a cloud-based processing component
        rospy.loginfo('[%s] Creating subscriber for topic %s',
                      self.state_name, self.cloud_obstacle_detection_topic)
        self.obstacle_sub = rospy.Subscriber(self.cloud_obstacle_detection_topic,
                                             ObjectList, self.store_detected_objects)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Subscriber for topic %s created',
                      self.state_name, self.cloud_obstacle_detection_topic)

        # we create a publisher for clearing the object cache of the cloud-based processing component
        rospy.loginfo('[%s] Creating publisher for topic %s',
                      self.state_name, self.cloud_obstacle_cache_reset_topic)
        self.obstacle_cache_reset_pub = rospy.Publisher(self.cloud_obstacle_cache_reset_topic,
                                                        Bool, queue_size=1)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Publisher for topic %s created',
                      self.state_name, self.cloud_obstacle_cache_reset_topic)

        self.tf_listener = tf.TransformListener()
