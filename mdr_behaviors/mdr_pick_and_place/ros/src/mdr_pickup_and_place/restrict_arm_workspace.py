#!/usr/bin/env python
#-*- encoding: utf-8 -*-
"""
Add boxes to planning scene to represent the workspace walls.
#TODO this should be replaced in the future with the octomap
Currently planning with octomap is too slow.
"""

__author__ = 'moriarty'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import moveit_msgs.msg
import tf

class ArmWorkspaceRestricter(object):
    """
    """
    def __init__(self):
        # params
        self.event_in = None
        self.is_restricted = False

        # node cycle rate (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.1)
        # frame which walls and workspace are added wrt
        self.wall_frame_id = rospy.get_param('~wall_frame_id', "/base_link")
        # wall height wrt wall_frame_id (m)
        self.wall_height = rospy.get_param('~wall_height', 0.38)
        # distance from wall_frame_id to side walls (m)
        self.wall_distance = rospy.get_param('~wall_distance', 0.37)
        # distance to platform from base wall_frame_id (m)
        self.platform_distance = rospy.get_param('~platform_distance', 0.4)
        # height of platform with respect to wall_frame_id (m)
        self.platform_height = rospy.get_param('~platform_height', 0.08)

        self.temp_distance_X = 0.8
        # publishers
        self.planning_scene_diff_publisher = rospy.Publisher('/planning_scene',
            moveit_msgs.msg.PlanningScene)
        rospy.sleep(2)


    def add_walls(self):
        self.add_box("restricter_left_wall",
            0.25, self.wall_distance, self.wall_height / 2.0,
            1.0, 0.04, self.wall_height)
        self.add_box("restricter_right_wall",
            0.25, -self.wall_distance, self.wall_height / 2.0,
            1.0, 0.04, self.wall_height)
        self.add_box("platform1",
            self.platform_distance, 0.0, 0.0,
            0.5 , self.wall_distance * 2.0, self.platform_height / 2.0)

        self.is_restricted = True

    def remove_walls(self):
        self.remove_box("restricter_left_wall")
        self.remove_box("restricter_right_wall")
        self.is_restricted = False

    def remove_shelf(self):
        self.remove_box("restricter_left_wall")
        self.remove_box("restricter_right_wall")
        self.remove_box("platform_1")
        self.remove_box("platform_2")
        self.remove_box("platform_3")
        self.remove_box("platform_4")
        self.remove_box("platform_5")
        self.is_restricted = False

    def add_shelfs(self):
        self.add_box("restricter_left_wall",
            -self.temp_distance_X, self.wall_distance, self.wall_height* 2.0 ,
            0.37, 0.04, self.wall_height* 5.0)
        self.add_box("restricter_right_wall",
            -self.temp_distance_X, -self.wall_distance, self.wall_height* 2.0 ,
            0.37, 0.04, self.wall_height* 5.0)
        self.add_box("platform_1",
            -self.temp_distance_X, 0.0, 0.12,
            0.37 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.add_box("platform_2",
                -self.temp_distance_X, 0.0, 0.12 + self.wall_height,
                0.37 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.add_box("platform_3",
                        -self.temp_distance_X, 0.0, 0.12 + self.wall_height*2 ,
                        0.37 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.add_box("platform_4",
                        -self.temp_distance_X, 0.0, 0.12 + self.wall_height*3,
                        0.37 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.add_box("platform_5",
                        -self.temp_distance_X, 0.0, 0.12 + self.wall_height*4,
                        0.37 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.is_restricted = True

    def add_table(self):
        self.add_box("platform_5", -self.temp_distance_X, 0.0, 0.76,
        0.76 , 1.8, 0.02)
        self.is_restricted = True

    def add_box(self, name, x, y, z, dx, dy, dz, yaw=0):
        """
        Adds two boxes to represent the walls
        """
        rospy.loginfo("Adding walls to moveit scene")

        box_object = moveit_msgs.msg.CollisionObject();
        box_object.header.frame_id = self.wall_frame_id
        box_object.id = name
        box_pose = geometry_msgs.msg.Pose()
        quat = tf.transformations.quaternion_from_euler(yaw, 0, 0)
        box_pose.orientation.w = quat[0]
        box_pose.orientation.x = quat[1]
        box_pose.orientation.y = quat[2]
        box_pose.orientation.z = quat[3]
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_shape = shape_msgs.msg.SolidPrimitive()
        box_shape.type = box_shape.BOX
        box_shape.dimensions.append(dx)
        box_shape.dimensions.append(dy)
        box_shape.dimensions.append(dz)
        box_object.primitives.append(box_shape)
        box_object.primitive_poses.append(box_pose)
        box_object.operation = box_object.ADD

        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects.append(box_object)
        planning_scene.is_diff = True

        self.planning_scene_diff_publisher.publish(planning_scene)

    def remove_box(self, name):
        """
        removes name from planning_scene
        """
        rospy.loginfo("removing wall from moveit scene")

        box_object = moveit_msgs.msg.CollisionObject();
        box_object.header.frame_id = self.wall_frame_id
        box_object.id = name

        box_object.operation = box_object.REMOVE

        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects.append(box_object)
        planning_scene.is_diff = True

        self.planning_scene_diff_publisher.publish(planning_scene)


def main():
    """
    Listens to event_in.
    When recieves "e_start", and walls haven't been added,
    they are added to planning scene.
    When recieves "e_stop" removes the walls which were added.
    """
    rospy.init_node("arm_workspace_restricter", anonymous=True)
    workspace_restricter = ArmWorkspaceRestricter()
    workspace_restricter.add_table()

if __name__ == '__main__':
    main()
