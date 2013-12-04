import rospy
import moveit_msgs.msg
import moveit_msgs.srv
import sensor_msgs.msg
import moveit_commander
from moveit_ros_planning_interface import _moveit_move_group_interface
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import sensor_msgs.msg


class MoveArm:

    def __init__(self, group_name):
        self.group_name = group_name
        self.commander = moveit_commander.MoveGroupCommander('arm')
        self.ac = actionlib.SimpleActionClient("/arm_controller/joint_trajectory_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction)
        self.ac.wait_for_server()


    def move(self, trajectory):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.commander.get_joints()
        goal.trajectory.header.frame_id = "base_link"
        goal.trajectory.header.stamp = rospy.Time.now()
        
        trajectory.insert(0, self.commander.get_current_joint_values())
        
        time = 0.0
        for i in xrange(len(trajectory)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = trajectory[i]
            if (i != 0):
                time += self.max_distance(trajectory[i - 1], trajectory[i]) * 5.0
            point.time_from_start = rospy.Duration(time)
            
            goal.trajectory.points.append(point)
        
        self.ac.send_goal(goal)
        self.ac.wait_for_result()


    def max_distance(self, p1, p2):
        max = 0.0
        for i in xrange(0, len(p1)):
            d = abs(p1[i] - p2[i])
            if (d > max): max = d
        return max