import time
import rospy
import moveit_commander

head = moveit_commander.MoveGroupCommander("head")
 
head.set_joint_value_target("head_tilt_joint", self.tilt_angle)
head.go()