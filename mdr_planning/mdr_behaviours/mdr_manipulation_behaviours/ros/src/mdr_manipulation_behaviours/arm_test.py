#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool

def main():
    # # Initialize moveit_commander and rospy
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_arm', anonymous=True)

    # # Create a RobotCommander object to interface with the robot
    # robot = moveit_commander.RobotCommander()

    # # Create a PlanningSceneInterface object to interface with the world surrounding the robot
    # scene = moveit_commander.PlanningSceneInterface()
 
    # group_name = "arm"
    # # Create a MoveGroupCommander object to interface with the arm of the robot
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    
    
    # planning_frame = move_group.get_planning_frame()
    # print("============ Planning frame: %s" % planning_frame)

    # # We can also print the name of the end-effector link for this group:
    # eef_link = move_group.get_end_effector_link()
    # print("============ End effector link: %s" % eef_link)

    # # We can get a list of all the groups in the robot:
    # group_names = robot.get_group_names()
    # print("============ Available Planning Groups:", robot.get_group_names())

    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot:
    # print("============ Printing robot state")
    # #print(robot.get_current_state())
    # print("random")

    # # Set the reference frame for the arm
    # move_group.set_pose_reference_frame('odom')

    # # Set the target pose for the arm
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.418000 # changing from 1.2
    # pose_goal.position.y = 0.078000
    # pose_goal.position.z = 0.742000
    # pose_goal.orientation.x = 0.758000
    # pose_goal.orientation.y = 0.000000
    # pose_goal.orientation.z = 0.652000
    # pose_goal.orientation.w = 1.000000

    # move_group.set_pose_target(pose_goal)

    # # Plan and execute the trajectory
    # plan = move_group.go(wait=True)

    # # Exit the script
    # moveit_commander.roscpp_shutdown()

    def callback(data):
    # Set the target pose for the arm
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.418000 # changing from 1.2
        pose_goal.position.y = 0.078000
        pose_goal.position.z = 0.742000
        pose_goal.orientation.x = 0.758000
        pose_goal.orientation.y = 0.000000
        pose_goal.orientation.z = 0.652000
        pose_goal.orientation.w = 1.000000

        move_group.set_pose_target(pose_goal)

        # Plan and execute the trajectory
        plan = move_group.go(wait=True)

        # Exit the script
        moveit_commander.roscpp_shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


if __name__ == '__main__':
    def callback(data):
    # Set the target pose for the arm
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.418000 # changing from 1.2
        pose_goal.position.y = 0.078000
        pose_goal.position.z = 0.742000
        pose_goal.orientation.x = 0.758000
        pose_goal.orientation.y = 0.000000
        pose_goal.orientation.z = 0.652000
        pose_goal.orientation.w = 1.000000

        move_group.set_pose_target(pose_goal)

        # Plan and execute the trajectory
        plan = move_group.go(wait=True)

        # Exit the script
        moveit_commander.roscpp_shutdown()
    try:
        rospy.init_node('custom_object_picker')
            # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_arm', anonymous=True)

        # Create a RobotCommander object to interface with the robot
        robot = moveit_commander.RobotCommander()

        # Create a PlanningSceneInterface object to interface with the world surrounding the robot
        scene = moveit_commander.PlanningSceneInterface()
    
        group_name = "arm"
        # Create a MoveGroupCommander object to interface with the arm of the robot
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
 
        # Set the reference frame for the arm
        move_group.set_pose_reference_frame('odom')
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.418000 # changing from 1.2
        pose_goal.position.y = 0.078000
        pose_goal.position.z = 0.742000
        pose_goal.orientation.x = 0.758000
        pose_goal.orientation.y = 0.000000
        pose_goal.orientation.z = 0.652000
        pose_goal.orientation.w = 1.000000

        move_group.set_pose_target(pose_goal)

        # Plan and execute the trajectory
        plan = move_group.go(wait=True)

        # Exit the script
        moveit_commander.roscpp_shutdown()
        # rospy.Subscriber("/pose/pose_soup", Bool, callback)
        # pub = rospy.Publisher('object_info', String, queue_size=10)
        rospy.loginfo("waiting for object pose....")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
