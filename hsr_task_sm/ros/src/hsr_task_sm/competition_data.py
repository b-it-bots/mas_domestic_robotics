#!/usr/bin/env python3
"""
German Open 2026 Competition Data Utility

Provides easy access to competition objects, locations, names, and mappings.

Usage:
    from hsr_task_sm.competition_data import CompetitionData
    
    data = CompetitionData()
    
    # Get all drinks
    drinks = data.get_objects_by_category('drinks')
    
    # Where should I put this apple?
    destination = data.get_destination_for_object('apple')  # -> 'couch_table'
    
    # What category is this?
    category = data.get_category('chips')  # -> 'snacks'
    
    # Get random guest name
    name = data.get_random_name()
"""

import os
import yaml
import random
import rospkg
import rospy
from typing import List, Dict, Optional


class CompetitionData:
    """
    Utility class for accessing German Open 2026 competition data.
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
        """Load competition data from YAML file."""
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('hsr_task_sm')
            config_path = os.path.join(pkg_path, 'ros', 'config', 'german_open_2026.yaml')
        except:
            # Fallback path
            config_path = os.path.expanduser(
                '~/catkin_ws/src/hsr_task_sm/ros/config/german_open_2026.yaml'
            )
        
        try:
            with open(config_path, 'r') as f:
                self._data = yaml.safe_load(f)
            rospy.loginfo(f'[CompetitionData] Loaded competition data from {config_path}')
        except Exception as e:
            rospy.logerr(f'[CompetitionData] Failed to load data: {e}')
            self._data = {}
    
    # =========================================================================
    # ROOMS
    # =========================================================================
    
    def get_rooms(self) -> List[str]:
        """Get all room names."""
        return self._data.get('rooms', [])
    
    # =========================================================================
    # LOCATIONS
    # =========================================================================
    
    def get_all_locations(self) -> List[str]:
        """Get all location names."""
        return self._data.get('all_locations', [])
    
    def get_placeable_locations(self) -> List[str]:
        """Get locations where objects can be placed."""
        locs = self._data.get('locations', {})
        return locs.get('placeable', [])
    
    def is_placeable(self, location: str) -> bool:
        """Check if objects can be placed at this location."""
        return location in self.get_placeable_locations()
    
    # =========================================================================
    # OBJECTS
    # =========================================================================
    
    def get_all_objects(self) -> List[str]:
        """Get all object names."""
        return self._data.get('all_objects', [])
    
    def get_objects_by_category(self, category: str) -> List[str]:
        """Get all objects in a category."""
        objects = self._data.get('objects', {})
        return objects.get(category, [])
    
    def get_categories(self) -> List[str]:
        """Get all object categories."""
        return list(self._data.get('objects', {}).keys())
    
    def get_category(self, object_name: str) -> Optional[str]:
        """Get the category of an object."""
        mapping = self._data.get('object_to_category', {})
        return mapping.get(object_name)
    
    def is_known_object(self, object_name: str) -> bool:
        """Check if object is in the known objects list."""
        return object_name in self.get_all_objects()
    
    # =========================================================================
    # MAPPINGS
    # =========================================================================
    
    def get_destination_for_object(self, object_name: str) -> Optional[str]:
        """Get the proper storage location for an object."""
        category = self.get_category(object_name)
        if category:
            mapping = self._data.get('category_to_destination', {})
            return mapping.get(category)
        return None
    
    def get_destination_for_category(self, category: str) -> Optional[str]:
        """Get the destination location for a category."""
        mapping = self._data.get('category_to_destination', {})
        return mapping.get(category)
    
    def get_category_for_location(self, location: str) -> Optional[str]:
        """Get what category belongs at a location."""
        mapping = self._data.get('location_categories', {})
        return mapping.get(location)
    
    # =========================================================================
    # NAMES (for HRI)
    # =========================================================================
    
    def get_names(self) -> List[str]:
        """Get all guest names."""
        return self._data.get('names', [])
    
    def get_random_name(self) -> str:
        """Get a random guest name."""
        names = self.get_names()
        return random.choice(names) if names else "Guest"
    
    def is_valid_name(self, name: str) -> bool:
        """Check if name is in the official list."""
        names = [n.lower() for n in self.get_names()]
        return name.lower() in names
    
    # =========================================================================
    # TEST OBJECTS
    # =========================================================================
    
    def get_test_objects(self, challenge: str = None) -> Dict[str, List[str]]:
        """Get test objects, optionally filtered by challenge."""
        test_objs = self._data.get('test_objects', {})
        if challenge:
            return {challenge: test_objs.get(challenge, [])}
        return test_objs
    
    # =========================================================================
    # DRINKS (commonly needed for HRI)
    # =========================================================================
    
    def get_drinks(self) -> List[str]:
        """Get all drink names (convenience method)."""
        return self.get_objects_by_category('drinks')
    
    def is_valid_drink(self, drink: str) -> bool:
        """Check if drink is in the official list."""
        drinks = [d.lower().replace('_', ' ') for d in self.get_drinks()]
        return drink.lower().replace('_', ' ') in drinks
    
    # =========================================================================
    # CLUSTERING (for tidy-up tasks)
    # =========================================================================
    
    def get_clustering_plan(self, detected_objects: List[str]) -> Dict[str, List[str]]:
        """
        Given a list of detected objects, return a plan of where to put them.
        
        Returns:
            Dict mapping destination -> list of objects to place there
        """
        plan = {}
        for obj in detected_objects:
            dest = self.get_destination_for_object(obj)
            if dest:
                if dest not in plan:
                    plan[dest] = []
                plan[dest].append(obj)
        return plan
    
    def cluster_objects(self, objects: List[str]) -> Dict[str, List[str]]:
        """
        Group objects by their category.
        
        Returns:
            Dict mapping category -> list of objects
        """
        clusters = {}
        for obj in objects:
            cat = self.get_category(obj)
            if cat:
                if cat not in clusters:
                    clusters[cat] = []
                clusters[cat].append(obj)
        return clusters


# Convenience function
def get_competition_data() -> CompetitionData:
    """Get the singleton CompetitionData instance."""
    return CompetitionData()


# Test when run directly
if __name__ == '__main__':
    data = CompetitionData()
    
    print("=== German Open 2026 Competition Data ===\n")
    
    print("ROOMS:", data.get_rooms())
    print()
    
    print("CATEGORIES:", data.get_categories())
    print()
    
    print("DRINKS:", data.get_drinks())
    print()
    
    print("Where to put 'apple'?", data.get_destination_for_object('apple'))
    print("Category of 'chips'?", data.get_category('chips'))
    print()
    
    print("Random guest name:", data.get_random_name())
    print()
    
    # Test clustering
    detected = ['apple', 'orange', 'coke', 'water', 'cup', 'bowl']
    print("Detected objects:", detected)
    print("Clustering plan:", data.get_clustering_plan(detected))
