import numpy as np
import tf
import rospy
import actionlib

from shapely.geometry import Polygon
from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker

from mas_perception_msgs.msg import ObjectList
from mdr_place_action.msg import PlaceAction, PlaceGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_wrs_tidy_up.utils import update_object_detection_params

class ReleaseObject(ScenarioStateBase):
    tf_listener = None
    object_detection_timeout_s = None
    cloud_object_detection_topic = None
    cloud_object_cache_reset_topic = None
    object_bounds_marker_topic = None
    cell_occupancy_marker_topic = None
    free_cell_center_marker_topic = None
    object_sub = None
    object_cache_reset_pub = None
    object_bounds_marker_pub = None
    cell_occupancy_marker_pub = None
    free_cell_center_marker_pub = None
    detected_cloud_objects = None
    last_cloud_object_detection_time = None
    cells = None
    cell_occupied = None
    num_cell_rows_per_tray = 3
    num_cell_cols_per_tray = 2
    planning_scene_map_file = None

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'release_object',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object',
                                               'environment_objects',
                                               'storage_location'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'release_object')
        self.action_server_name = kwargs.get('action_server_name', 'place_server')
        self.release_timeout = kwargs.get('release_timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

        self.object_detection_timeout_s = kwargs.get('object_detection_timeout', 5.)
        self.cloud_object_detection_topic = kwargs.get('cloud_object_detection_topic',
                                                         '/mas_perception/cloud_obstacle_detection/obstacle_objects')
        self.cloud_object_cache_reset_topic = kwargs.get('cloud_object_cache_reset_topic',
                                                           '/mas_perception/cloud_obstacle_detection/reset_cache')
        self.object_bounds_marker_topic = kwargs.get('object_bounds_marker_topic', 'object_bounds_2d')
        self.cell_occupancy_marker_topic = kwargs.get('cell_occupancy_marker_topic', 'tray_cells_occupancy')
        self.free_cell_center_marker_topic = kwargs.get('free_cell_center_marker_topic', 'free_tray_cell_centers')
        self.num_cell_rows_per_tray = kwargs.get('num_cell_rows_per_tray', 3)
        self.num_cell_cols_per_tray = kwargs.get('num_cell_cols_per_tray', 1)
        self.planning_scene_map_file = kwargs.get('planning_scene_map_file', '')
        self.__init_ros_components()

    def execute(self, userdata):
        if self.cells is None:
            self.__init_cells(userdata.environment_objects)

        goal = PlaceGoal()
        goal.release_on_impact = False

        # storage_location = userdata.storage_location.lower()
        storage_location = 'tray_1'
        if storage_location.find('bin') != -1:
            release_target = userdata.environment_objects[storage_location]
            goal.pose = self.get_bin_release_pose(release_target, userdata.grasped_object)
        elif storage_location.find('tray') != -1:
            goal.pose = self.get_tray_release_pose()
        else:
            rospy.logerr('Storage location %s unknown', userdata.storage_location)
            return 'failed_after_retrying'

        client = actionlib.SimpleActionClient(self.action_server_name, PlaceAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(self.release_timeout)))
        result = client.get_result()
        if result:
            if result.success:
                rospy.loginfo('[%s] Successfully released object', self.state_name)
                return 'succeeded'
            else:
                rospy.logerr('[%s] Could not release object', self.state_name)
        else:
            rospy.logerr('[%s] Object not released within %f seconds; giving up',
                         self.state_name, self.release_timeout)
            client.cancel_all_goals()

        if self.retry_count == self.number_of_retries:
            rospy.logerr('[%s] Could not release object after retrying; giving up', self.state_name)
            self.retry_count = 0
            return 'failed_after_retrying'

        rospy.loginfo('[%s] Retrying to release object', self.state_name)
        self.retry_count += 1
        return 'succeeded'

    def get_bin_release_pose(self, release_target, grasped_object):
        candidate_pose = PoseStamped()
        candidate_pose.header.stamp = rospy.Time(0)
        candidate_pose.header.frame_id = release_target.pose.header.frame_id

        candidate_pose.pose.position.x = release_target.pose.pose.position.x
        candidate_pose.pose.position.y = release_target.pose.pose.position.y
        candidate_pose.pose.position.z = release_target.pose.pose.position.z + \
                                         (release_target.dimensions.vector.z / 2)
        candidate_pose.pose.position.z += 0.2

        # use top-down orientation for throwing
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id = 'base_link'
        gripper_pose.header.stamp = rospy.Time(0)

        gripper_quaternion = tf.transformations.quaternion_from_euler(np.pi, 0., 0.)
        gripper_pose.pose.orientation.x = gripper_quaternion[0]
        gripper_pose.pose.orientation.y = gripper_quaternion[1]
        gripper_pose.pose.orientation.z = gripper_quaternion[2]
        gripper_pose.pose.orientation.w = gripper_quaternion[3]

        gripper_pose_in_target_frame = self.tf_listener.transformPose(release_target.pose.header.frame_id, gripper_pose)
        candidate_pose.pose.orientation = gripper_pose_in_target_frame.pose.orientation
        return candidate_pose

    def get_tray_release_pose(self):
        if self.last_cloud_object_detection_time is None:
            self.last_cloud_object_detection_time = rospy.Time.now().to_sec()
        last_msg_time = self.last_cloud_object_detection_time

        # Always reset the previously detected objects at execution start
        self.detected_cloud_objects = None

        rospy.loginfo('[%s] Updating cloud object detection params for "trays" surface', self.state_name)
        update_object_detection_params('trays')

        rospy.loginfo('[%s] Resetting cloud object cache and waiting a bit', self.state_name)
        self.object_cache_reset_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        rospy.loginfo('[%s] Waiting for cloud object detection', self.state_name)
        timeout_reached = False
        waiting_start_time = rospy.Time.now().to_sec()
        while abs(last_msg_time - self.last_cloud_object_detection_time) < 1e-5 and \
              not timeout_reached:
            rospy.sleep(0.05)
            if (rospy.Time.now().to_sec() - waiting_start_time) > self.object_detection_timeout_s:
                timeout_reached = True

        self.cell_occupied = self.find_occupied_cells(self.detected_cloud_objects)

        if self.detected_cloud_objects:
            rospy.logwarn('[%s] Detected %d objects', self.state_name, len(self.detected_cloud_objects))
            rospy.logwarn('[%s] Occupied cells: %d', self.state_name, self.cell_occupied.count(True))
            self.publish_object_bound_markers(self.detected_cloud_objects)
        else:
            rospy.loginfo('[%s] No objects detected on trays. All cells are free', self.state_name)

        free_tray_cell_pos = self.get_free_cell_centers()
        self.publish_cell_occupancy_markers(self.cell_occupied)
        self.publish_free_cell_center_markers(free_tray_cell_pos)

        furthest_cell = free_tray_cell_pos[np.argmin(free_tray_cell_pos[:,1])]

        candidate_pose = PoseStamped()
        candidate_pose.header.stamp = rospy.Time(0)
        candidate_pose.header.frame_id = 'map'

        candidate_pose.pose.position.x = furthest_cell[0]
        candidate_pose.pose.position.y = furthest_cell[1]
        candidate_pose.pose.position.z = 0.55

        # use top-down orientation for throwing
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id = 'base_link'
        gripper_pose.header.stamp = rospy.Time(0)

        gripper_quaternion = tf.transformations.quaternion_from_euler(np.pi, 0., 0.)
        gripper_pose.pose.orientation.x = gripper_quaternion[0]
        gripper_pose.pose.orientation.y = gripper_quaternion[1]
        gripper_pose.pose.orientation.z = gripper_quaternion[2]
        gripper_pose.pose.orientation.w = gripper_quaternion[3]

        gripper_pose_in_target_frame = self.tf_listener.transformPose('map', gripper_pose)
        candidate_pose.pose.orientation = gripper_pose_in_target_frame.pose.orientation
        return candidate_pose

    def find_occupied_cells(self, obj_list):
        occupied_cells = [False] * self.cells.shape[0]
        if obj_list and len(obj_list) > 0:
            for cell_id in range(len(occupied_cells)):
                cell_bounds = self.cells[cell_id]
                for obj in obj_list:
                    if self.object_in_cell(obj, cell_bounds):
                        occupied_cells[cell_id] = True
                        break
        return occupied_cells

    def object_in_cell(self, obj, cell_bounds, min_overlap_ratio=0.15):
        bbox_2d, _ = self.get_2d_bounding_box(obj, 'map')
        bounds_polygon = Polygon(bbox_2d)
        cell_min = cell_bounds[:2]
        cell_max = cell_bounds[2:]
        cell_polygon = Polygon([(cell_min[0], cell_min[1]),
                                (cell_min[0], cell_max[1]),
                                (cell_max[0], cell_max[1]),
                                (cell_max[0], cell_min[1])])
        bounds_polygon.intersection(cell_polygon).area/cell_polygon.area
        return bounds_polygon.intersects(cell_polygon) and \
               bounds_polygon.intersection(cell_polygon).area/cell_polygon.area > min_overlap_ratio

    def get_free_cell_centers(self):
        centers = []
        for cell_id in range(self.cells.shape[0]):
            if not self.cell_occupied[cell_id]:
                centers.append(self.get_cell_center(cell_id))
        return np.array(centers)

    def store_detected_objects(self, object_msg):
        '''Saves the objects received in the input message and stores the
        current time as the latest object detection time.

        Keyword arguments:
        object_msg: mas_perception_msgs.msg.ObjectList -- list of detected objects
        '''
        self.detected_cloud_objects = object_msg.objects
        self.last_cloud_object_detection_time = rospy.Time.now().to_sec()

    def publish_object_bound_markers(self, obj_list):
        if obj_list:
            markers = MarkerArray()
            for i, obj in enumerate(obj_list):
                markers.markers.append(self.get_bounds_marker(obj, i))
            self.object_bounds_marker_pub.publish(markers)

    def publish_cell_occupancy_markers(self, occupied_cells):
        if occupied_cells:
            markers = MarkerArray()
            for i in range(len(occupied_cells)):
                markers.markers.append(self.get_cell_marker(self.cells[i], i, occupied=occupied_cells[i]))
            self.cell_occupancy_marker_pub.publish(markers)

    def publish_free_cell_center_markers(self, free_cell_centers):
        if free_cell_centers.shape[0] > 0:
            markers = MarkerArray()
            for i in range(free_cell_centers.shape[0]):
                markers.markers.append(self.get_cell_center_marker(free_cell_centers[i], i))
            self.free_cell_center_marker_pub.publish(markers)

    def get_2d_bounding_box(self, obj, target_frame=None):
        obj_pose = obj.pose
        if target_frame:
            obj_pose = self.tf_listener.transformPose(target_frame, obj_pose)
        pos = np.array([obj_pose.pose.position.x, obj_pose.pose.position.y])
        orientation = obj_pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(orientation_list)[2]
        bbox_half_dim = np.array([obj.bounding_box.dimensions.x, obj.bounding_box.dimensions.y]) * 0.5

        transformation_matrix = np.array([[np.cos(yaw), -np.sin(yaw),   pos[0]],
                                          [np.sin(yaw), np.cos(yaw),    pos[1]],
                                          [0,           0,              1     ]])
        top_left = transformation_matrix.dot(np.array([-bbox_half_dim[0], bbox_half_dim[1], 1.0]))[:2]
        top_right = transformation_matrix.dot(np.array([bbox_half_dim[0], bbox_half_dim[1], 1.0]))[:2]
        bottom_right = transformation_matrix.dot(np.array([bbox_half_dim[0], -bbox_half_dim[1], 1.0]))[:2]
        bottom_left = transformation_matrix.dot(np.array([-bbox_half_dim[0], -bbox_half_dim[1], 1.0]))[:2]

        return [top_left, top_right, bottom_right, bottom_left], obj_pose.pose.position.z

    def get_bounds_marker(self, obj, id):
        bbox_2d, z = self.get_2d_bounding_box(obj)
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        # marker.lifetime = rospy.Duration.from_sec(1.0)
        marker.header.frame_id = obj.pose.header.frame_id
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.ns = ""
        marker.id = id
        marker.color = ColorRGBA(0, 0, 1, 1)

        for i in range(len(bbox_2d)):
            s, e = Point(), Point()

            s.x = bbox_2d[i - 1][0]
            s.y = bbox_2d[i - 1][1]
            s.z = z
            marker.points.append(s)

            e.x = bbox_2d[i][0]
            e.y = bbox_2d[i][1]
            e.z = z
            marker.points.append(e)

        return marker

    def get_cell_marker(self, cell_bounds, id, occupied=False, z=0.4):
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        # marker.lifetime = rospy.Duration.from_sec(1.0)
        marker.header.frame_id = "map"
        marker.scale.x = 0.015 if occupied else 0.005
        marker.scale.y = 0.005
        marker.ns = ""
        marker.id = id
        marker.color = ColorRGBA(1, 0, 0, 1) if occupied else ColorRGBA(0, 1, 0, 1)

        min = [cell_bounds[0], cell_bounds[1]]
        max = [cell_bounds[2], cell_bounds[3]]
        marker.points.append(Point(x=min[0], y=min[1], z=z))
        marker.points.append(Point(x=min[0], y=max[1], z=z))
        marker.points.append(Point(x=min[0], y=max[1], z=z))
        marker.points.append(Point(x=max[0], y=max[1], z=z))
        marker.points.append(Point(x=max[0], y=max[1], z=z))
        marker.points.append(Point(x=max[0], y=min[1], z=z))
        marker.points.append(Point(x=max[0], y=min[1], z=z))
        marker.points.append(Point(x=min[0], y=min[1], z=z))

        return marker

    def get_cell_center_marker(self, cell_center, id, z=0.4):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        # marker.lifetime = rospy.Duration.from_sec(1.0)
        marker.header.frame_id = "map"
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.ns = ""
        marker.id = id
        marker.color = ColorRGBA(0, 1, 0, 1)
        marker.pose.position = Point(x=cell_center[0], y=cell_center[1], z=z)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def __init_cells(self, environment_objects):
        self.cells = self.__create_cells(environment_objects)
        self.cell_occupied = [False] * self.cells.shape[0]

    def __create_cells(self, environment_objects):
        t1_min, t1_max, t2_min, t2_max = self.__get_tray_bounds(environment_objects)
        t1_width = t1_max[0] - t1_min[0]
        t1_len = t1_max[1] - t1_min[1]
        t2_width = t2_max[0] - t2_min[0]
        t2_len = t2_max[1] - t2_min[1]

        t1_cell_size = np.array([t1_width/self.num_cell_cols_per_tray,
                                 t1_len/self.num_cell_rows_per_tray])
        t2_cell_size = np.array([t2_width/self.num_cell_cols_per_tray,
                                 t2_len/self.num_cell_rows_per_tray])
        t1_cells = []
        t2_cells = []
        for row in range(self.num_cell_rows_per_tray):
            for col in range(self.num_cell_cols_per_tray):
                t1 = t1_min + np.array([col * t1_cell_size[0], row * t1_cell_size[1]])
                t2 = t2_min + np.array([col * t2_cell_size[0], row * t2_cell_size[1]])
                t1_cells.append(np.array([t1[0], t1[1], t1[0]+t1_cell_size[0], t1[1]+t1_cell_size[1]]))
                t2_cells.append(np.array([t2[0], t2[1], t2[0]+t2_cell_size[0], t2[1]+t2_cell_size[1]]))

        tray_cells = np.array([t2_cells[0]])
        for cell in range(1, len(t2_cells)):
            tray_cells = np.vstack((tray_cells, t2_cells[cell]))
        for cell in range(len(t1_cells)):
            tray_cells = np.vstack((tray_cells, t1_cells[cell]))

        return tray_cells

    def __get_tray_bounds(self, environment_objects):
        tray_1, tray_2 = None, None
        tray_1_center, tray_2_center = None, None
        tray_1_dim, tray_2_dim = None, None
        for obj_name, obj in environment_objects.items():
            if tray_1 and tray_2:
                break
            elif tray_1 is None and obj_name == 'tray_1':
                tray_1 = obj
                tray_1_center = np.array([obj.bounding_box.center.x, obj.bounding_box.center.y])
                tray_1_dim = np.array([obj.bounding_box.dimensions.x, obj.bounding_box.dimensions.y])
            elif tray_2 is None and obj_name == 'tray_2':
                tray_2 = obj
                tray_2_center = np.array([obj.bounding_box.center.x, obj.bounding_box.center.y])
                tray_2_dim = np.array([obj.bounding_box.dimensions.x, obj.bounding_box.dimensions.y])

        tray_1_min = tray_1_center + (tray_1_dim / 2.0)
        tray_1_max = tray_1_center - (tray_1_dim / 2.0)
        tray_2_min = tray_2_center + (tray_2_dim / 2.0)
        tray_2_max = tray_2_center - (tray_2_dim / 2.0)

        return [tray_1_min, tray_1_max, tray_2_min, tray_2_max]

    def get_cell_center(self, cell_id):
        cell_min = self.cells[cell_id][:2]
        cell_max = self.cells[cell_id][2:]
        return cell_min + ((cell_max - cell_min) / 2.0)

    def __init_ros_components(self):
        '''Initialises:
        * an object detection action client
        * a subscriber for objects detected by a cloud-based processing component
        * a publisher for resetting the object cache of the cloud-based processing component
        '''
        # we create a subscriber for objects detected by a cloud-based processing component
        rospy.loginfo('[%s] Creating subscriber for topic %s',
                      self.state_name, self.cloud_object_detection_topic)
        self.object_sub = rospy.Subscriber(self.cloud_object_detection_topic,
                                             ObjectList, self.store_detected_objects)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Subscriber for topic %s created',
                      self.state_name, self.cloud_object_detection_topic)

        # we create a publisher for clearing the object cache of the cloud-based processing component
        rospy.loginfo('[%s] Creating publisher for topic %s',
                      self.state_name, self.cloud_object_cache_reset_topic)
        self.object_cache_reset_pub = rospy.Publisher(self.cloud_object_cache_reset_topic,
                                                        Bool, queue_size=1)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Publisher for topic %s created',
                      self.state_name, self.cloud_object_cache_reset_topic)

        # we create a publisher for publishing the 2D oriented bounding boxes of the detected objects on the trays
        rospy.loginfo('[%s] Creating publisher for topic %s',
                      self.state_name, self.object_bounds_marker_topic)
        self.object_bounds_marker_pub = rospy.Publisher(self.object_bounds_marker_topic,
                                                        MarkerArray, queue_size=1)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Publisher for topic %s created',
                      self.state_name, self.object_bounds_marker_topic)

        # we create a publisher for publishing the occupancy status of all the tray cells
        rospy.loginfo('[%s] Creating publisher for topic %s',
                      self.state_name, self.cell_occupancy_marker_topic)
        self.cell_occupancy_marker_pub = rospy.Publisher(self.cell_occupancy_marker_topic,
                                                         MarkerArray, queue_size=1)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Publisher for topic %s created',
                      self.state_name, self.cell_occupancy_marker_topic)

        # we create a publisher for publishing the centers if the free tray cells
        rospy.loginfo('[%s] Creating publisher for topic %s',
                      self.state_name, self.free_cell_center_marker_topic)
        self.free_cell_center_marker_pub = rospy.Publisher(self.free_cell_center_marker_topic,
                                                         MarkerArray, queue_size=1)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Publisher for topic %s created',
                      self.state_name, self.free_cell_center_marker_topic)

        self.tf_listener = tf.TransformListener()
