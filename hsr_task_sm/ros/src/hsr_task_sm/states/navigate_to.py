#!/usr/bin/env python3
"""
NavigateTo smach state.

Sends a named destination to /move_base_server via actionlib.
No ROSPlan KB involved.

German Open 2026 Navigation Goals:
    See: ros/config/navigation_goals.yaml
    
    Available locations:
    - home, door1_inside, door1_outside, door2_entrance
    - bedroom_enterance, bed_right, bedside_table, bedroom_table
    - living_room_table, living_room_far, living_room_left, living_room_tv
    - dining_table, dining_table_right, dining_table_left, dining_table_far
    - dish_washer, fridge, cabinet, shelf, washing_machine
    
    Aliases: entrance, kitchen, bedroom, living_room, etc.
"""

import rospy
import smach
import actionlib

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal


class NavigateTo(smach.State):
    """
    Navigate to a named location using move_base action.
    
    Supports:
    - Direct destination string: NavigateTo(destination='kitchen')
    - Userdata key: NavigateTo(destination_key='target_location')
    
    Outcomes:
        succeeded              -- destination reached
        failed                 -- navigation failed; will retry up to `retries` times
        failed_after_retrying  -- max retries exhausted
    """

    def __init__(self,
                 destination=None,
                 destination_key=None,
                 server_name='move_base_server',
                 timeout=20.0,
                 retries=2):
        """
        Args:
            destination: Direct destination name (e.g., 'kitchen', 'entrance')
            destination_key: Userdata key containing destination name
            server_name: Action server name
            timeout: Navigation timeout in seconds
            retries: Number of retries on failure
        """
        input_keys = [destination_key] if destination_key else []
        
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=input_keys
        )
        self.destination = destination
        self.destination_key = destination_key
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0
        self.server_name = server_name
        self.client = None

    def _ensure_client(self):
        """Create action client if not exists."""
        if self.client is None:
            rospy.loginfo('[NavigateTo] Connecting to %s ...', self.server_name)
            self.client = actionlib.SimpleActionClient(self.server_name, MoveBaseAction)
            self.client.wait_for_server(rospy.Duration(10.0))
            rospy.loginfo('[NavigateTo] Connected.')

    def execute(self, userdata):
        self._ensure_client()
        
        # Get destination from parameter or userdata
        if self.destination_key and hasattr(userdata, self.destination_key):
            dest = getattr(userdata, self.destination_key)
        else:
            dest = self.destination
        
        if not dest:
            rospy.logerr('[NavigateTo] No destination specified')
            return 'failed_after_retrying'
        
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.NAMED_TARGET
        goal.destination_location = dest

        rospy.loginfo('[NavigateTo] Navigating to "%s" ...', dest)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        if not finished:
            self.client.cancel_goal()
            rospy.logerr('[NavigateTo] Timed out after %.0fs', self.timeout)
            return self._retry()

        result = self.client.get_result()
        if result and result.success:
            rospy.loginfo('[NavigateTo] Reached "%s"', dest)
            self.retry_count = 0
            return 'succeeded'

        rospy.logwarn('[NavigateTo] Failed to reach "%s"', dest)
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[NavigateTo] Max retries (%d) reached', self.retries)
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[NavigateTo] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'


class NavigateToPose(smach.State):
    """
    Navigate to a specific pose (x, y, theta) using move_base action.
    
    Supports:
    - Direct pose: NavigateToPose(x=1.0, y=2.0, theta=0.0)
    - From userdata: NavigateToPose(pose_key='target_pose')
    - From navigation_goals.yaml: NavigateToPose(location='dining_table')
    
    Outcomes:
        succeeded, failed, failed_after_retrying
    """
    
    def __init__(self,
                 x=None, y=None, theta=None,
                 pose_key=None,
                 location=None,
                 server_name='move_base_server',
                 timeout=120.0,
                 retries=2):
        """
        Args:
            x, y, theta: Direct coordinates
            pose_key: Userdata key containing [x, y, theta] list
            location: Location name from navigation_goals.yaml
            server_name: Action server name
            timeout: Navigation timeout
            retries: Retry count
        """
        input_keys = [pose_key] if pose_key else []
        
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=input_keys
        )
        
        self.x = x
        self.y = y
        self.theta = theta
        self.pose_key = pose_key
        self.location = location
        self.server_name = server_name
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0
        self.client = None
        self.nav_goals = None
    
    def _ensure_client(self):
        if self.client is None:
            self.client = actionlib.SimpleActionClient(self.server_name, MoveBaseAction)
            self.client.wait_for_server(rospy.Duration(10.0))
    
    def _get_pose(self, userdata):
        """Get pose from various sources."""
        # From userdata
        if self.pose_key and hasattr(userdata, self.pose_key):
            pose = getattr(userdata, self.pose_key)
            if isinstance(pose, (list, tuple)) and len(pose) >= 3:
                return pose[0], pose[1], pose[2]
        
        # From direct parameters
        if self.x is not None and self.y is not None:
            return self.x, self.y, self.theta or 0.0
        
        # From navigation_goals.yaml
        if self.location:
            if self.nav_goals is None:
                try:
                    from hsr_task_sm.navigation_goals import get_navigation_goals
                    self.nav_goals = get_navigation_goals()
                except ImportError:
                    rospy.logwarn('[NavigateToPose] Could not load navigation_goals')
                    return None
            
            goal = self.nav_goals.get_goal(self.location)
            if goal:
                return goal
        
        return None
    
    def execute(self, userdata):
        self._ensure_client()
        
        pose = self._get_pose(userdata)
        if not pose:
            rospy.logerr('[NavigateToPose] Could not determine target pose')
            return 'failed_after_retrying'
        
        x, y, theta = pose
        
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.POSE
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        
        # Convert theta to quaternion
        import tf.transformations as tft
        from geometry_msgs.msg import Quaternion
        q = tft.quaternion_from_euler(0, 0, theta)
        goal.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        rospy.loginfo('[NavigateToPose] Going to (%.2f, %.2f, %.2f)', x, y, theta)
        self.client.send_goal(goal)
        
        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        if not finished:
            self.client.cancel_goal()
            rospy.logerr('[NavigateToPose] Timeout')
            return self._retry()
        
        result = self.client.get_result()
        if result and result.success:
            rospy.loginfo('[NavigateToPose] Arrived')
            self.retry_count = 0
            return 'succeeded'
        
        return self._retry()
    
    def _retry(self):
        self.retry_count += 1
        if self.retry_count > self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        return 'failed'
