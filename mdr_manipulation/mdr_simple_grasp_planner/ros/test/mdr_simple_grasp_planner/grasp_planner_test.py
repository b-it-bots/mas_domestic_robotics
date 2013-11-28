#!/usr/bin/env python

import rospy
import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import mdr_simple_grasp_planner.grasp_planner
import unittest

class TestGraspPlanner(unittest.TestCase):

    def test_plan(self):
        planner = mdr_simple_grasp_planner.grasp_planner.GraspPlanner()
        grasps = planner.plan()
        
        self.assertGreater(len(grasps), 0)


if __name__ == '__main__':
    unittest.main()