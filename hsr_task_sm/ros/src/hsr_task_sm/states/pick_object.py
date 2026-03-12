#!/usr/bin/env python3
"""
PickObject smach state — uses pickup_server (mdr_pickup_action).

Sends a TOP_GRASP goal with the object pose in base_link frame.
The z coordinate is set to table_z + z_offset so the gripper
approaches above the table surface, not at the object centroid.

Sequence:
  1. Find closest object on relevant plane
  2. Transform pose to base_link, set z = table_z + z_offset
  3. Send PickupGoal (TOP_GRASP) to pickup_server
  4. Report success/failure
"""

import rospy
import smach
import actionlib
import numpy as np
import tf

from geometry_msgs.msg import PoseStamped
from mdr_pickup_action.msg import PickupAction, PickupGoal


class PickObject(smach.State):
    """
    Outcomes:
        succeeded
        failed
        failed_after_retrying
        find_objects_before_picking
    """

    def __init__(self,
                 picking_surface_prefix='table',
                 pickup_server='pickup_server',
                 z_offset=0.05,
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying',
                      'find_objects_before_picking'],
            input_keys=['perceived_planes'],
            output_keys=['grasped_object', 'grasped_object_height']
        )
        self.picking_surface_prefix = picking_surface_prefix
        self.z_offset = z_offset
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[PickObject] Connecting to %s ...', pickup_server)
        self.pickup_client = actionlib.SimpleActionClient(pickup_server, PickupAction)
        self.pickup_client.wait_for_server()
        rospy.loginfo('[PickObject] pickup_server connected.')

        self.tf_listener = tf.TransformListener()

    # ------------------------------------------------------------------
    def _select_strategy(self, obj):
        """
        Choose SIDEWAYS_GRASP or TOP_GRASP based on object shape and dimensions.

        Rules (in priority order):
          1. Cylinder shape           → SIDEWAYS (gripper wraps around)
          2. height > max(width,depth)→ SIDEWAYS (tall object)
          3. width/depth > 2x height  → TOP      (flat/plate-like)
          4. default                  → SIDEWAYS  (safer fallback)
        """
        shape_name = ''
        if hasattr(obj, 'shape') and obj.shape:
            shape_name = obj.shape.shape.lower()

        # Get bounding box dimensions (x=width, y=depth, z=height)
        width, depth, height = 0.0, 0.0, 0.0
        if hasattr(obj, 'bounding_box') and obj.bounding_box and \
                hasattr(obj.bounding_box, 'dimensions'):
            d = obj.bounding_box.dimensions
            width, depth, height = d.x, d.y, d.z
        elif hasattr(obj, 'dimensions') and obj.dimensions:
            d = obj.dimensions.vector
            width, depth, height = d.x, d.y, d.z

        if shape_name == 'cylinder':
            strategy = PickupGoal.SIDEWAYS_GRASP
            reason = 'cylinder shape'
        elif height > 0 and width > 0 and height > max(width, depth):
            strategy = PickupGoal.SIDEWAYS_GRASP
            reason = 'tall object (h=%.3f > w=%.3f)' % (height, max(width, depth))
        elif height > 0 and width > 0 and max(width, depth) > 2.0 * height:
            strategy = PickupGoal.TOP_GRASP
            reason = 'flat object (w=%.3f > 2x h=%.3f)' % (max(width, depth), height)
        else:
            strategy = PickupGoal.SIDEWAYS_GRASP
            reason = 'default fallback'

        strategy_name = 'SIDEWAYS' if strategy == PickupGoal.SIDEWAYS_GRASP else 'TOP'
        rospy.loginfo('[PickObject] Grasp strategy: %s (%s)', strategy_name, reason)
        return strategy

    # ------------------------------------------------------------------
    def _select_closest(self, planes):
        best_obj, best_plane = None, None
        min_dist = float('inf')
        for plane in planes:
            for obj in plane.object_list.objects:
                p = obj.pose.pose.position
                dist = np.sqrt(p.x**2 + p.y**2 + p.z**2)
                if dist < min_dist:
                    min_dist = dist
                    best_obj = obj
                    best_plane = plane
        return best_obj, best_plane

    def _retry(self, outcome):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[PickObject] Retry %d/%d', self.retry_count, self.retries)
        return outcome

    # ------------------------------------------------------------------
    def execute(self, userdata):
        planes = userdata.perceived_planes.planes
        relevant = [p for p in planes
                    if self.picking_surface_prefix in p.name
                    and len(p.object_list.objects) > 0]

        if not relevant:
            rospy.logwarn('[PickObject] No objects on "%s" planes', self.picking_surface_prefix)
            return 'find_objects_before_picking'

        best_obj, best_plane = self._select_closest(relevant)
        if best_obj is None:
            return 'find_objects_before_picking'

        # Transform object pose to base_link
        obj_ps = PoseStamped()
        obj_ps.header = best_obj.pose.header
        obj_ps.pose   = best_obj.pose.pose         # PoseStamped -> Pose
        try:
            obj_pose_bl = self.tf_listener.transformPose('base_link', obj_ps)
        except Exception as e:
            rospy.logerr('[PickObject] TF transform failed: %s', e)
            return self._retry('failed')

        # Get table z in base_link
        plane_ps = PoseStamped()
        plane_ps.header = best_plane.header
        plane_ps.pose.position.x = best_plane.plane_point.x
        plane_ps.pose.position.y = best_plane.plane_point.y
        plane_ps.pose.position.z = best_plane.plane_point.z
        plane_ps.pose.orientation.w = 1.0
        try:
            plane_bl = self.tf_listener.transformPose('base_link', plane_ps)
            table_z = plane_bl.pose.position.z
        except Exception as e:
            rospy.logwarn('[PickObject] Plane TF failed, using raw z: %s', e)
            table_z = best_plane.plane_point.z

        # Build goal pose: use actual object pose from perception
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = obj_pose_bl.pose.position.x
        grasp_pose.pose.position.y = obj_pose_bl.pose.position.y
        grasp_pose.pose.position.z = table_z + self.z_offset
        grasp_pose.pose.orientation.w = 1.0

        rospy.loginfo('[PickObject] "%s" -> base_link x=%.3f y=%.3f table_z=%.3f grasp_z=%.3f',
                      best_obj.name,
                      grasp_pose.pose.position.x,
                      grasp_pose.pose.position.y,
                      table_z,
                      grasp_pose.pose.position.z)

        goal = PickupGoal()
        goal.pose     = grasp_pose
        goal.strategy = self._select_strategy(best_obj)

        self.pickup_client.send_goal(goal)
        finished = self.pickup_client.wait_for_result(rospy.Duration(self.timeout))

        if not finished:
            self.pickup_client.cancel_goal()
            rospy.logwarn('[PickObject] pickup_server timed out after %.1fs', self.timeout)
            return self._retry('failed')

        result = self.pickup_client.get_result()
        if result is None or not result.success:
            rospy.logwarn('[PickObject] pickup_server returned failure')
            return self._retry('find_objects_before_picking')

        rospy.loginfo('[PickObject] Grasped "%s"', best_obj.name)
        userdata.grasped_object = best_obj.name
        
        # Store object height for placing (from bounding box if available)
        obj_height = 0.05  # default 5cm
        if hasattr(best_obj, 'bounding_box') and best_obj.bounding_box:
            bb = best_obj.bounding_box
            # bounding_box.dimensions is a Vector3 (x, y, z)
            if hasattr(bb, 'dimensions'):
                obj_height = bb.dimensions.z  # z is height
            elif hasattr(bb, 'z'):
                obj_height = bb.z
        rospy.loginfo('[PickObject] Object height: %.3f m', obj_height)
        userdata.grasped_object_height = obj_height
        
        self.retry_count = 0
        return 'succeeded'
