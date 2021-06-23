import numpy as np

import rospy
import actionlib
import tf

from geometry_msgs.msg import Pose, PoseStamped
from mas_perception_msgs.msg import ObjectList
from mdr_manipulation_msgs.srv import UpdatePlanningScene, UpdatePlanningSceneRequest
from mdr_pickup_action.msg import PickupAction, PickupGoal

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class PickupObject(ScenarioStateBase):
    pickup_server_name = 'pickup_server'
    pickup_goal_pose_topic = '/pickup_server/goal_pose'
    pickup_client = None
    goal_pose_pub = None
    tf_listener = None
    grasping_timeout_s = 30.
    grasping_height_offset = 0.
    planning_scene_update_service_name = ''
    planning_scene_update_proxy = None

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'pickup_object',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                                   input_keys=['selected_object',
                                               'detected_objects',
                                               'environment_objects'],
                                   output_keys=['grasped_object'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'pickup_object')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.pickup_server_name = kwargs.get('pickup_server_name', 'pickup_server')
        self.pickup_goal_pose_topic = kwargs.get('pickup_goal_pose_topic',
                                                 '/pickup_server/goal_pose')
        self.grasping_timeout_s = kwargs.get('grasping_timeout_s', 30.)
        self.grasping_height_offset = kwargs.get('grasping_height_offset', 0.05)
        self.planning_scene_update_service_name = kwargs.get('planning_scene_update_service_name',
                                                             '/move_arm_action/update_planning_scene')
        self.retry_count = 0
        self.__init_ros_components()

    def execute(self, userdata):
        self.reset_planning_scene(userdata.environment_objects)
        object_to_pick_up = userdata.selected_object
        objects_except_target = [obj for obj in userdata.detected_objects
                                 if obj.name != object_to_pick_up.name]
        self.update_planning_scene(objects_except_target, UpdatePlanningSceneRequest.ADD)

        goal = PickupGoal()
        goal.pose.header.frame_id = object_to_pick_up.pose.header.frame_id
        goal.pose.header.stamp = rospy.Time.now()

        grasping_pose, grasping_strategy = self.get_grasping_pose_and_strategy(object_to_pick_up)
        goal.pose.pose = grasping_pose
        goal.strategy = grasping_strategy

        rospy.loginfo('Publishing grasping pose on topic %s', self.pickup_goal_pose_topic)
        self.goal_pose_pub.publish(goal.pose)

        rospy.loginfo('[%s] Picking up object at %s position (%f %f %f)', self.state_name,
                      goal.pose.header.frame_id, goal.pose.pose.position.x,
                      goal.pose.pose.position.y, goal.pose.pose.position.z)

        self.pickup_client.send_goal(goal)
        if self.pickup_client.wait_for_result(rospy.Duration.from_sec(self.grasping_timeout_s)):
            pickup_result = self.pickup_client.get_result()
            if pickup_result.success:
                rospy.loginfo('[%s] Successfully grasped object %s', self.state_name, object_to_pick_up.name)
                userdata.grasped_object = object_to_pick_up
                self.update_planning_scene(objects_except_target, UpdatePlanningSceneRequest.REMOVE)
                return 'succeeded'
            else:
                rospy.logerr('[%s] Failed to grasp object', self.state_name)
        else:
            rospy.logerr('[%s] Object not picked up within %f seconds; giving up',
                         self.state_name, self.grasping_timeout_s)
            self.pickup_client.cancel_all_goals()

        if self.retry_count == self.number_of_retries:
            rospy.logerr('[%s] Could not pick up object after retrying; giving up', self.state_name)
            self.retry_count = 0
            self.update_planning_scene(objects_except_target, UpdatePlanningSceneRequest.REMOVE)
            return 'failed_after_retrying'

        rospy.loginfo('[%s] Retrying to place object', self.state_name)
        self.retry_count += 1
        self.update_planning_scene(objects_except_target, UpdatePlanningSceneRequest.REMOVE)
        return 'failed'

    def reset_planning_scene(self, environment_objects):
        object_list = ObjectList()
        object_list.objects = [environment_objects[name] for name in environment_objects]

        # initialising the MoveIt! planning scene
        update_planning_scene_req = UpdatePlanningSceneRequest()
        update_planning_scene_req.operation = UpdatePlanningSceneRequest.ADD
        update_planning_scene_req.objects = object_list

        rospy.loginfo('[%s] Initialising planning scene', self.state_name)
        response = self.planning_scene_update_proxy(update_planning_scene_req)
        if response is not None:
            if response.success:
                rospy.loginfo('[%s] Successfully updated the planning scene', self.state_name)
            else:
                rospy.logerr('[%s] Failed to update the planning scene', self.state_name)
        else:
            rospy.logerr('[%s] Response not received', self.state_name)

    def update_planning_scene(self, objects, operation):
        object_list = ObjectList()
        object_list.objects = objects

        update_planning_scene_req = UpdatePlanningSceneRequest()
        update_planning_scene_req.objects = object_list
        update_planning_scene_req.operation = operation
        if operation == UpdatePlanningSceneRequest.ADD:
            rospy.loginfo('[%s] Adding static objects to planning scene', self.state_name)
        elif operation == UpdatePlanningSceneRequest.REMOVE:
            rospy.loginfo('[%s] Removing static objects from planning scene', self.state_name)

        response = self.planning_scene_update_proxy(update_planning_scene_req)
        if response is not None:
            if response.success:
                rospy.loginfo('[%s] Successfully updated the planning scene', self.state_name)
            else:
                rospy.logerr('[%s] Failed to update the planning scene', self.state_name)
        else:
            rospy.logerr('[%s] Response not received', self.state_name)

    def get_grasping_pose_and_strategy(self, object_to_pick_up):
        '''Returns a geometry_msgs.msg.Pose object representing a grasping pose
        for the given object. The grasping position is given by the center of the
        object's bounding box; the orientation ensures a top-down grasp aligned
        with the longest axis of the object.

        Keyword arguments:
        object_to_pick_up: mas_perception_msgs.msg.Object

        '''
        pose = Pose()
        pose.position.x = object_to_pick_up.bounding_box.center.x
        pose.position.y = object_to_pick_up.bounding_box.center.y
        pose.position.z = object_to_pick_up.bounding_box.center.z

        grasping_strategy = None
        # this orientation guarantees a sideways grasp and
        # alignment along the longest axis of the object
        if object_to_pick_up.dimensions.vector.z > max(object_to_pick_up.dimensions.vector.x,
                                                       object_to_pick_up.dimensions.vector.y):
            desired_gripper_orientation_base_link = (np.pi, -np.pi/2, 0.)
            grasping_strategy = PickupGoal.SIDEWAYS_GRASP
        # this orientation guarantees a top-down grasp and
        # alignment along the longest axis of the object
        else:
            object_pose_in_base_link = self.tf_listener.transformPose('base_link', object_to_pick_up.pose)
            euler_orientation = tf.transformations.euler_from_quaternion([object_pose_in_base_link.pose.orientation.x,
                                                                          object_pose_in_base_link.pose.orientation.y,
                                                                          object_pose_in_base_link.pose.orientation.z,
                                                                          object_pose_in_base_link.pose.orientation.w])
            gripper_orientation_z = euler_orientation[2]

            desired_gripper_orientation_base_link = (np.pi, 0, gripper_orientation_z)
            grasping_strategy = PickupGoal.TOP_GRASP

            # we set the grasping pose along z to be the top of the object to prevent
            # the robot pushing down the object with the gripper
            pose.position.z += (object_to_pick_up.dimensions.vector.z / 2) + self.grasping_height_offset

        pose.orientation = self.get_gripper_orientation(desired_gripper_orientation_base_link,
                                                        object_to_pick_up.pose.header.frame_id)

        return pose, grasping_strategy

    def get_gripper_orientation(self, orientation, target_frame):
        '''Returns a geometry_msgs.msg.Quaternion object representing the given
        orientation, which is represented in "base_link", in the given target frame.

        Keyword arguments:
        orientation: Tuple[float, float, float] -- desired (roll, pitch, yaw) orientation of
                                                   the gripper in the base_link frame
        target_frame: str -- frame in which the orientation should be represented

        '''
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id = 'base_link'
        gripper_pose.header.stamp = rospy.Time(0)

        gripper_quaternion = tf.transformations.quaternion_from_euler(*orientation)
        gripper_pose.pose.orientation.x = gripper_quaternion[0]
        gripper_pose.pose.orientation.y = gripper_quaternion[1]
        gripper_pose.pose.orientation.z = gripper_quaternion[2]
        gripper_pose.pose.orientation.w = gripper_quaternion[3]

        rospy.loginfo('[%s] Transforming pose from base_link to %s', self.state_name, target_frame)
        gripper_pose_in_target_frame = self.tf_listener.transformPose(target_frame, gripper_pose)
        return gripper_pose_in_target_frame.pose.orientation

    def __init_ros_components(self):
        rospy.loginfo('Initialising client for action %s', self.pickup_server_name)
        self.pickup_client = actionlib.SimpleActionClient(self.pickup_server_name, PickupAction)
        self.pickup_client.wait_for_server()
        rospy.loginfo('Client for action %s initialised', self.pickup_server_name)

        rospy.loginfo('[%s] Creating a service proxy for %s',
                      self.state_name, self.planning_scene_update_service_name)
        rospy.wait_for_service(self.planning_scene_update_service_name)
        self.planning_scene_update_proxy = rospy.ServiceProxy(self.planning_scene_update_service_name,
                                                              UpdatePlanningScene)
        rospy.loginfo('[%s] Service proxy for %s created',
                      self.state_name, self.planning_scene_update_service_name)

        rospy.loginfo('Initialising publisher for goal pose on topic %s', self.pickup_goal_pose_topic)
        self.goal_pose_pub = rospy.Publisher(self.pickup_goal_pose_topic, PoseStamped, queue_size=1)
        rospy.loginfo('Publisher for topic %s initialised', self.pickup_goal_pose_topic)

        self.tf_listener = tf.TransformListener()
