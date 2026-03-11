#!/usr/bin/env python3
"""
Navigation Goals Utility

Provides easy access to navigation goal points for German Open 2026.
Integrates with the mdr_move_base_action for named target navigation.

Usage:
    from hsr_task_sm.navigation_goals import NavigationGoals
    
    nav = NavigationGoals()
    
    # Get goal coordinates
    x, y, theta = nav.get_goal('entrance')
    
    # Get all goals in a room
    kitchen_goals = nav.get_room_goals('kitchen')
    
    # Get challenge-specific locations
    hri_locs = nav.get_challenge_locations('hri_challenge')
"""

import os
import yaml
import rospkg
import rospy
from typing import List, Dict, Tuple, Optional
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations as tft


class NavigationGoals:
    """
    Utility class for accessing navigation goal points.
    """
    
    _instance = None
    _data = None
    
    def __new__(cls):
        """Singleton pattern - only load data once."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._load_data()
        return cls._instance
    
    def _load_data(self):
        """Load navigation goals from YAML file."""
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('hsr_task_sm')
            config_path = os.path.join(pkg_path, 'ros', 'config', 'navigation_goals.yaml')
        except:
            config_path = os.path.expanduser(
                '~/catkin_ws/src/hsr_task_sm/ros/config/navigation_goals.yaml'
            )
        
        try:
            with open(config_path, 'r') as f:
                self._data = yaml.safe_load(f)
            rospy.loginfo(f'[NavigationGoals] Loaded from {config_path}')
        except Exception as e:
            rospy.logerr(f'[NavigationGoals] Failed to load: {e}')
            self._data = {'navigation_goals': {}, 'location_aliases': {}}
    
    # =========================================================================
    # GOAL RETRIEVAL
    # =========================================================================
    
    def get_goal(self, location_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get goal coordinates for a location.
        
        Args:
            location_name: Name of location (can be alias)
        
        Returns:
            Tuple (x, y, theta) or None if not found
        """
        # Check if it's an alias
        aliases = self._data.get('location_aliases', {})
        actual_name = aliases.get(location_name, location_name)
        
        # Get from navigation goals
        goals = self._data.get('navigation_goals', {})
        coords = goals.get(actual_name)
        
        if coords and len(coords) >= 3:
            return (coords[0], coords[1], coords[2])
        
        rospy.logwarn(f'[NavigationGoals] Unknown location: {location_name}')
        return None
    
    def get_pose_stamped(self, location_name: str, frame_id: str = 'map') -> Optional[PoseStamped]:
        """
        Get a PoseStamped message for a location.
        
        Args:
            location_name: Name of location
            frame_id: Reference frame (default: 'map')
        
        Returns:
            PoseStamped or None
        """
        coords = self.get_goal(location_name)
        if not coords:
            return None
        
        x, y, theta = coords
        
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = tft.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose
    
    def get_all_goals(self) -> Dict[str, List[float]]:
        """Get all navigation goals."""
        return self._data.get('navigation_goals', {})
    
    def get_goal_names(self) -> List[str]:
        """Get list of all goal names (including aliases)."""
        goals = set(self._data.get('navigation_goals', {}).keys())
        aliases = set(self._data.get('location_aliases', {}).keys())
        return sorted(goals | aliases)
    
    # =========================================================================
    # ROOM-BASED QUERIES
    # =========================================================================
    
    def get_room_goals(self, room_name: str) -> List[str]:
        """Get all goal names in a room."""
        rooms = self._data.get('rooms', {})
        room = rooms.get(room_name, {})
        return room.get('locations', [])
    
    def get_room_center(self, room_name: str) -> Optional[Tuple[float, float]]:
        """Get approximate center of a room."""
        rooms = self._data.get('rooms', {})
        room = rooms.get(room_name, {})
        center = room.get('center')
        if center and len(center) >= 2:
            return (center[0], center[1])
        return None
    
    def get_room_for_location(self, location_name: str) -> Optional[str]:
        """Find which room a location belongs to."""
        # Resolve alias first
        aliases = self._data.get('location_aliases', {})
        actual_name = aliases.get(location_name, location_name)
        
        rooms = self._data.get('rooms', {})
        for room_name, room_data in rooms.items():
            if actual_name in room_data.get('locations', []):
                return room_name
        return None
    
    # =========================================================================
    # CHALLENGE-SPECIFIC
    # =========================================================================
    
    def get_challenge_locations(self, challenge_name: str) -> Dict[str, str]:
        """Get location mappings for a specific challenge."""
        challenges = self._data.get('challenges', {})
        return challenges.get(challenge_name, {})
    
    def get_challenge_goal(self, challenge_name: str, location_key: str) -> Optional[Tuple[float, float, float]]:
        """Get a goal coordinate for a challenge-specific location."""
        locations = self.get_challenge_locations(challenge_name)
        actual_name = locations.get(location_key)
        if actual_name:
            return self.get_goal(actual_name)
        return None
    
    # =========================================================================
    # UTILITIES
    # =========================================================================
    
    def resolve_alias(self, name: str) -> str:
        """Resolve a location alias to actual goal name."""
        aliases = self._data.get('location_aliases', {})
        return aliases.get(name, name)
    
    def is_valid_location(self, name: str) -> bool:
        """Check if a location name is valid (goal or alias)."""
        return name in self.get_goal_names()
    
    def get_nearest_goal(self, x: float, y: float) -> Optional[str]:
        """Find the nearest named goal to a position."""
        goals = self.get_all_goals()
        min_dist = float('inf')
        nearest = None
        
        for name, coords in goals.items():
            if len(coords) >= 2:
                dist = ((coords[0] - x) ** 2 + (coords[1] - y) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    nearest = name
        
        return nearest


# Convenience function
def get_navigation_goals() -> NavigationGoals:
    """Get the singleton NavigationGoals instance."""
    return NavigationGoals()


# Test when run directly
if __name__ == '__main__':
    nav = NavigationGoals()
    
    print("=== German Open 2026 Navigation Goals ===\n")
    
    print("All goal names:")
    print(nav.get_goal_names())
    print()
    
    print("Goal for 'entrance':", nav.get_goal('entrance'))
    print("Goal for 'kitchen':", nav.get_goal('kitchen'))
    print("Goal for 'dishwasher':", nav.get_goal('dishwasher'))
    print()
    
    print("Kitchen room goals:", nav.get_room_goals('kitchen'))
    print()
    
    print("HRI challenge locations:")
    for k, v in nav.get_challenge_locations('hri_challenge').items():
        coords = nav.get_goal(v)
        print(f"  {k}: {v} -> {coords}")
    print()
    
    print("Pick & Place destinations:")
    for k, v in nav.get_challenge_locations('pick_place').items():
        if 'destination' in k:
            coords = nav.get_goal(v)
            print(f"  {k}: {v} -> {coords}")
