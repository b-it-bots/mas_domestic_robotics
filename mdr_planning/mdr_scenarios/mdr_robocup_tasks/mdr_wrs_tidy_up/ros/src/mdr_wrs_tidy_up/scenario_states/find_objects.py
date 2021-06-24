import uuid
import numpy as np
from shapely.geometry import Point, Polygon

import rospy
from geometry_msgs.msg import PoseArray
import tf
import actionlib
from std_msgs.msg import Bool

from mas_perception_msgs.msg import ObjectList, DetectObjectsAction, DetectObjectsGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class TaskContext(object):
    CLEAN_UP = 'clean_up'
    GO_AND_GET_IT = 'go_and_get_it'


def get_plane_polygon(center_position, dimensions):
    p1 = (center_position.x - (dimensions.x / 2), center_position.y - (dimensions.y / 2))
    p2 = (center_position.x - (dimensions.x / 2), center_position.y + (dimensions.y / 2))
    p3 = (center_position.x + (dimensions.x / 2), center_position.y + (dimensions.y / 2))
    p4 = (center_position.x + (dimensions.x / 2), center_position.y - (dimensions.y / 2))
    return Polygon((p1, p2, p3, p4))

def find_closest_object(obj, obj_list):
    min_dist = 1e10
    min_dist_class = ''
    for x in obj_list:
        relative_position = np.array([obj.pose.pose.position.x - x.pose.pose.position.x,
                                      obj.pose.pose.position.y - x.pose.pose.position.y,
                                      obj.pose.pose.position.z - x.pose.pose.position.z])
        dist = np.linalg.norm(relative_position)
        if dist < min_dist:
            min_dist = dist
            min_dist_class = x.category
    return min_dist_class

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
    large_object_size_threshold = None
    large_object_height_threshold = None
    large_object_aspect_ratio_threshold = None
    filtered_poses_topic = None
    filtered_poses_pub = None

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
        self.object_detection_timeout_s = kwargs.get('object_detection_timeout', 5.)
        self.object_detection_server_name = kwargs.get('object_detection_server_name',
                                                       '/mas_perception/detect_objects')
        self.task_context = kwargs.get('task_context', TaskContext.CLEAN_UP)
        self.cloud_obstacle_detection_topic = kwargs.get('cloud_obstacle_detection_topic',
                                                         '/mas_perception/cloud_obstacle_detection/obstacle_objects')
        self.cloud_obstacle_cache_reset_topic = kwargs.get('cloud_obstacle_cache_reset_topic',
                                                           '/mas_perception/cloud_obstacle_detection/reset_cache')
        self.max_allowed_obj_height_cm = kwargs.get('max_allowed_obj_height_cm', 0.15)
        self.min_allowed_dist_to_leg = kwargs.get('min_allowed_dist_to_leg', 0.15)
        self.tables_to_clean_up = kwargs.get('tables_to_clean_up', ['long_table_b', 'tall_table'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.large_object_size_threshold = kwargs.get('large_object_size_threshold', 0.15)
        self.large_object_height_threshold = kwargs.get('large_object_height_threshold', 0.075)
        self.large_object_aspect_ratio_threshold = kwargs.get('large_object_aspect_ratio_threshold', 0.4)
        self.filtered_poses_topic = kwargs.get('filtered_poses_topic', 'filtered_object_poses')
        self.filtered_poses_pub = rospy.Publisher(self.filtered_poses_topic, PoseArray, queue_size=1)
        self.__init_ros_components()

    def execute(self, userdata):
        last_msg_time = self.last_cloud_object_detection_time

        detected_cam_objects = []
        self.object_detection_client.send_goal(DetectObjectsGoal())
        detection_start_time = rospy.Time.now().to_sec()
        if self.object_detection_client.wait_for_result(rospy.Duration.from_sec(self.object_detection_timeout_s)):
            result = self.object_detection_client.get_result()
            detected_cam_objects = result.objects.objects
            detected_cam_obj_labels = [obj.category for obj in detected_cam_objects]
            rospy.loginfo('[{0}] Detected cam object labels: {1}'.format(self.state_name, detected_cam_obj_labels))
        else:
            rospy.logerr('[%s] No objects detected within %f seconds; proceeding with cloud object detection',
                         self.state_name, self.object_detection_timeout_s)
        detection_end_time = rospy.Time.now().to_sec()
        detection_elapsed_time = detection_end_time - detection_start_time
        rospy.loginfo('[%s] Elapsed detection time: %s', self.state_name, detection_elapsed_time)

        rospy.loginfo('[%s] Resetting cloud obstacle cache and waiting a bit', self.state_name)
        self.obstacle_cache_reset_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        rospy.loginfo('[%s] Waiting for cloud obstacle detection', self.state_name)
        timeout_reached = False
        waiting_start_time = rospy.Time.now().to_sec()
        while abs(last_msg_time - self.last_cloud_object_detection_time) < 1e-5 and \
              not timeout_reached:
            rospy.sleep(0.05)
            if (rospy.Time.now().to_sec() - waiting_start_time) > self.object_detection_timeout_s:
                timeout_reached = True

        if timeout_reached:
            rospy.loginfo('[%s] No objects detected within %f seconds; giving up',
                          self.state_name, self.object_detection_timeout_s)
            current_location = userdata.destination_locations[0]
            if userdata.object_location == 'floor':
                userdata.floor_objects_cleared[current_location] = True
            elif userdata.object_location == 'table':
                userdata.table_objects_cleared[current_location] = True
            return 'no_objects'

        rospy.loginfo('[%s] Detected %d objects', self.state_name, len(self.detected_cloud_objects))

        # workaround for large objects (such as the pitcher) sticking to the gripper in Gazebo
        filtered_objects = self.filter_objects_by_height(self.detected_cloud_objects)
        if userdata.object_location == 'floor':
            filtered_objects = self.filter_objects_by_dist_to_table_legs(filtered_objects,
                                                                         userdata.environment_objects)
            filtered_objects = self.filter_objects_under_tables(filtered_objects,
                                                                userdata.environment_objects)
            filtered_objects = self.filter_large_objects(filtered_objects)

        if self.task_context == TaskContext.GO_AND_GET_IT:
            filtered_objects = self.filter_objects_outside_bounds(self.detected_cloud_objects)

        userdata.detected_objects = self.label_detected_cloud_objects(detected_cam_objects, filtered_objects)

        # if no objects are seen in the current view, we register the location as "cleared"
        if not filtered_objects:
            current_location = userdata.destination_locations[0]
            if userdata.object_location == 'floor':
                userdata.floor_objects_cleared[current_location] = True
            elif userdata.object_location == 'table':
                userdata.table_objects_cleared[current_location] = True
            return 'no_objects'
        else:
            pose_array = PoseArray()
            pose_array.header.frame_id = filtered_objects[0].pose.header.frame_id
            pose_array.header.stamp = rospy.Time.now()
            for obj in filtered_objects:
                pose_array.poses.append(obj.pose.pose)
            self.filtered_poses_pub.publish(pose_array)

        return 'succeeded'

    def store_detected_objects(self, object_msg):
        '''Saves the objects received in the input message and stores the
        current time as the latest object detection time.

        Keyword arguments:
        object_msg: mas_perception_msgs.msg.ObjectList -- list of detected objects
        '''
        self.detected_cloud_objects = object_msg.objects
        self.last_cloud_object_detection_time = rospy.Time.now().to_sec()

    def label_detected_cloud_objects(self, detected_cam_objects, cloud_objects):
        for i, obj in enumerate(cloud_objects):
            closest_object_label = find_closest_object(obj, detected_cam_objects)
            obj.name = '{0}-{1}'.format(closest_object_label, str(uuid.uuid4()))
            obj.category = closest_object_label
            rospy.loginfo('[%s] Assigned label "%s" to detected cloud object at position (%f, %f, %f)',
                            self.state_name, closest_object_label, obj.pose.pose.position.x,
                            obj.pose.pose.position.y, obj.pose.pose.position.z)
        return cloud_objects

    def filter_objects_by_height(self, objects):
        rospy.loginfo('[%s] Filtering detected objects using height threshold %f',
                      self.state_name, self.max_allowed_obj_height_cm)
        filtered_objects = [obj for obj in objects
                            if obj.dimensions.vector.z <= self.max_allowed_obj_height_cm]
        rospy.loginfo('[%s] Keeping %d objects', self.state_name, len(filtered_objects))
        return filtered_objects

    def filter_objects_outside_bounds(self, objects):
        rospy.loginfo('[%s] Filtering objects that are close to walls in corridor', self.state_name)
        filtered_objects = []
        for obj in objects:
            if obj.pose.pose.position.y < 2.2 or obj.pose.pose.position.y > 3.0:
                # bounds: (2.05, 3.15)
                filtered_objects.append(obj)
            else:
                print('Filtered object:')
                print(obj.name)
                print(obj.pose)
        return filtered_objects

    def transform_pose(self, pose_msg):
        transformed_pose = None
        while not rospy.is_shutdown():
            try:
                transformed_pose = self.tf_listener.transformPose('map', pose_msg)
                break
            except:
                continue

        return transformed_pose

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

    def filter_objects_under_tables(self, objects, environment_objects):
        rospy.loginfo('[%s] Filtering detected under tables %s',
                      self.state_name, self.tables_to_clean_up)

        table_plane_polygons = []
        for table_name in self.tables_to_clean_up:
            table_obj = environment_objects[table_name]
            plane_polygon = get_plane_polygon(table_obj.pose.pose.position,
                                              table_obj.dimensions.vector)
            table_plane_polygons.append(plane_polygon)

        polygon_convex_hull = None
        if table_plane_polygons:
            polygon_union = table_plane_polygons[0]
            for polygon in table_plane_polygons:
                polygon_union = polygon_union.union(polygon)
            polygon_convex_hull = polygon_union.convex_hull

        point_in_any_polygon = lambda point: sum([1 for polygon in table_plane_polygons
                                                  if polygon.contains(point)]) > 0

        point_in_union_convex_hull = lambda point: False if polygon_convex_hull is None \
                                                         else polygon_convex_hull.contains(point)

        filtered_objects = []
        for obj in objects:
            obj_point = Point(obj.pose.pose.position.x, obj.pose.pose.position.y)
            if not point_in_any_polygon(obj_point) and not point_in_union_convex_hull(obj_point):
                filtered_objects.append(obj)

        rospy.loginfo('[%s] Keeping %d objects', self.state_name, len(filtered_objects))
        return filtered_objects

    def filter_large_objects(self, objects):
        rospy.loginfo('[%s] Filtering large objects with params: max_size(%f), max_height(%f), max_aspect_ratio(%f)',
                      self.state_name, self.large_object_size_threshold,
                      self.large_object_height_threshold,
                      self.large_object_aspect_ratio_threshold)

        filtered_objects = []
        for obj in objects:
            if obj.dimensions.vector.x <= self.large_object_size_threshold and \
               obj.dimensions.vector.y <= self.large_object_size_threshold:
               # The object is not considered as a large object
               filtered_objects.append(obj)
            else:
                # Found a large object
                if obj.dimensions.vector.z <= self.large_object_height_threshold:
                    dimensions = obj.bounding_box.dimensions
                    obb_aspect_ratio_xy = dimensions.y / dimensions.x if dimensions.y < dimensions.x else \
                                          dimensions.x / dimensions.y
                    if obb_aspect_ratio_xy <= self.large_object_aspect_ratio_threshold:
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
