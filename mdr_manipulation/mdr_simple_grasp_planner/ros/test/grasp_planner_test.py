#!/usr/bin/env python

import rospy
import mdr_simple_grasp_planner.grasp_planner
import unittest
import rostest

PKG = 'mdr_simple_grasp_planner'


class TestGraspPlanner(unittest.TestCase):

    def test_plan(self):
        planner = mdr_simple_grasp_planner.grasp_planner.GraspPlanner()
        grasps = planner.plan()
        self.assertGreater(len(grasps), 0)


if __name__ == '__main__':
    rospy.init_node('test_grasp_planner')
    rostest.rosrun(PKG, 'test_grasp_planner', TestGraspPlanner)
