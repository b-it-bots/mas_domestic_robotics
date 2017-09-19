#!/usr/bin/env python

import rospy
import actionlib
import simple_script_server
import geometry_msgs.msg
import tf
import math

import mdr_behavior_msgs.srv
import kinematics
import move_arm


class SimplePlace:

    def __init__(self):
        self.kinematics = kinematics.Kinematics('arm')
        self.move_arm = move_arm.MoveArm('arm')

        # distance from object center to pre-grasp
        self.pre_grasp_distance = rospy.get_param('~pre_grasp_distance')
        # distance from object to grasp
        self.grasp_distance = rospy.get_param('~grasp_distance')
        # height of the post-grasp over the object center
        self.post_grasp_height = rospy.get_param('~post_grasp_height')
        # at which angle of approach to start searching for a solution
        self.orbit_start_angle = 30.0 * math.pi / 180.0;

        rospy.loginfo('Pre-grasp distance: %f', self.pre_grasp_distance)
        rospy.loginfo('Grasp distance: %f', self.grasp_distance)
        rospy.loginfo('Post-grasp height: %f', self.post_grasp_height)

        self.sss = simple_script_server.simple_script_server()

        # action clients
        #rospy.loginfo('Waiting for 'grasp_posture_control' action')
        #self.hand_action = actionlib.SimpleActionClient('grasp_posture_control', object_manipulation_msgs.msg.GraspHandPostureExecutionAction)
        #self.hand_action.wait_for_server()
        #rospy.loginfo('Found action 'grasp_posture_control'')

        # service servers
        self.place_server = rospy.Service('place', mdr_behavior_msgs.srv.Place, self.place)
        rospy.loginfo('"place" service advertised')


    def place(self, req):
        '''
        Perform the place task

        :param req: mdr_behavior_msgs.PlaceRequest
            req.position: geometry_msgs.msg.PointStamped
        '''
        rospy.logdebug('Start planning trajectory')

        req.position.point.y += 0.05

        # pre-calculate the trajectory to the position
        traj = self.plan_trajectory(req.position)

        if (not traj):
            rospy.loginfo('No trajectory found')
            res = mdr_behavior_msgs.srv.PlaceResponse()
            res.success = False
            return res

        rospy.logdebug('Found a trajectory')

        # move the arm along the calculated trajectory to the grasp
        #self.sss.move('arm', traj[0:len(traj)-2], blocking = True)
        self.move_arm.move(traj[0:len(traj) - 2])

        # open the hand
        sdh_handle = self.sss.move('sdh', 'cylopen')

        # move to post-grasp
        rospy.logdebug('Moving to post-grasp')
        #self.sss.move('arm', traj[len(traj)-2:len(traj)], blocking = True)
        self.move_arm.move(traj[len(traj) - 2:len(traj)])
        self.sss.move('sdh', 'cylclosed')

        res = mdr_behavior_msgs.srv.PlaceResponse()
        res.success = True
        rospy.loginfo('Grasped the object successfully')

        return res


    def plan_trajectory(self, position):
        '''
        Create a trajectory from the current position to the grasp position. An
        intermediate pre-grasp in front of the object is inserted between
        current and goal position.

        :param position: geometry_msgs.msg.PointStamped
        :return: None or ([Float[7], ..., Float[7]]
        '''

        # extract the the pre-grasp pose from the parameter server
        pregrasp_param = rospy.get_param('/script_server/arm/home')
        #prepregrasp_param = rospy.get_param('/script_server/arm/prepregrasp')
        #prepregrasp = prepregrasp_param[0]
        init_configuration = pregrasp_param[0]

        prepregrasp = [-1.5091513691305107, -1.6491849487892085, -2.3696059954222668, -1.6160414652896298, 0.52644578912550133, 1.9561455454345944, -2.3656075221199249]

        angle_counter = -0.2
        while (angle_counter < math.pi):
            angle_counter += 0.2

            # first we try to find a solution that is oriented with the hand's
            # pre-grasp pose later grasps try to approach opposite of that angle
            offset_angle_counter = angle_counter + self.orbit_start_angle
            if (offset_angle_counter >= math.pi / 2.0):
                angle = math.pi / 2.0 + self.orbit_start_angle - offset_angle_counter
            else:
                angle = offset_angle_counter

            rospy.loginfo('Planning grasp with approach angle: %f', angle)

            # determine the trajectory to the pre-grasp pose
            traj_pregrasp = self.calculate_trajectory_abs(position, self.pre_grasp_distance, angle, 0.1, prepregrasp)
            if (not traj_pregrasp):
                continue

            rospy.loginfo('Found pre-grasp')

            # determine the trajectory to the grasp pose
            traj_grasp = self.calculate_trajectory_abs(position, self.grasp_distance, angle, 0.0, traj_pregrasp)
            if (not traj_grasp):
                continue

            rospy.loginfo('Found grasp')

            # determine the trajectory to the post-grasp pose
            traj_post_grasp = self.calculate_trajectory_abs(position, self.grasp_distance, angle, self.post_grasp_height, traj_grasp)
            if (not traj_post_grasp):
                continue

            rospy.loginfo('Found post-grasp')

            # if all trajectories have been determined we return the result
            return [list(prepregrasp), list(traj_post_grasp), list(traj_grasp), list(traj_pregrasp), list(prepregrasp)]

        return None


    def calculate_trajectory_abs(self, position, distance, angle, height, configuration):
        '''
        Based on the goal position, the distance and angle from the goal
        position and a given configuration determine the joint angles to reach
        the distance in front of the provided position.

        :param position: geometry_msgs.msg.PointStamped
        :param distance: Float
        :param angle: Float
        :param height: Float
        :param configuration: Float[7]
        :return: None or Float[7]
        '''
        pose = geometry_msgs.msg.PoseStamped()
        pose = self.calculate_pose_abs(position, distance, angle, height)

        # determine the inverse kinematics solution
        return self.kinematics.inverse_kinematics(pose, configuration)



    def calculate_pose_abs(self, position, distance, angle, height):
        '''
        Determine a position that is the given distance in front of the provided
        position. The angle is given in the object frame and describes the angle
        of approach to the provided position.

        :param position: geometry_msgs.msg.PointStamped
        :param distance: Float
        :param angle: Float
        :param height: Float
        :return: geometry_msgs.msg.PoseStamped
        '''

        pose = geometry_msgs.msg.PoseStamped()
        pose.header = position.header

        deltaX = distance * math.cos(angle)
        deltaY = distance * math.sin(angle)
        roll = 0.0
        pitch = -math.pi / 2.0
        yaw = angle

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose.pose.position.x = position.point.x + deltaX
        pose.pose.position.y = position.point.y + deltaY
        pose.pose.position.z = position.point.z + height
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


def main():
    rospy.init_node('place')

    place = SimplePlace()

    rospy.spin()
