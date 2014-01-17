import rospy
import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import tf
import math
import numpy

class GraspPlanner:
    
    def __init__(self):
        self.joint_names = [
            'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint',
            'sdh_finger_21_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint',
            'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
        self.cylindrical_open = [
                        0, -0.9854, 0.9472, 0, -0.9854, 0.9472, -0.9854, 0.9472]
        self.cylindrical_closed = [
                        0, 0, 1.0472, 0, 0, 1.0472, 0, 1.0472]
        self.spherical_open = [
                        1.047, -0.785, 1.047, 0, -0.785, 1.047, -0.785, 1.047]
        self.spherical_closed = [
                        1.047, -0.262, 1.047, 0, -0.262, 1.047, -0.262, 1.047]
        
        # The number of generated grasps in one orbit
        self.samples_per_orbit = 8
        
        # The radius (in meters) of the orbit, i.e. the distance of the hand's
        # reference frame to the object's reference frame in the x-y plane
        self.grasp_distance = 0.15
        
        self.pregrasp_distance = 0.35
        
        # The name of the gripper's reference coordinate system
        self.gripper_link = "arm_7_link"


    def plan(self):
        grasps = []
        grasps.extend(self.generate_grasps_from_top(self.grasp_distance,
                                                    self.pregrasp_distance))
        grasps.extend(self.generate_grasps_from_side(self.grasp_distance,
                                                     self.pregrasp_distance))
        
        return grasps


    def generate_grasps_from_side(self, grasp_distance, pregrasp_distance):
        '''
        :param grasp_distance: The distance from the object to the wrist's
        coordinate system in the final grasp.
        :type distance: float
        
        :param pregrasp_distance: The distance from the object to the wrist's
        coordinate system for the pre-grasp.
        :type distance: float
        
        :return: A list of generated grasps
        :rtype: moveit_msgs.msg.Grasp[]
        '''
        grasps = []
        for wrist_roll in [0, math.pi]:
            for angle_iter in range(0, 2 * int(self.samples_per_orbit)):
                # angle is measured about z-axis in base link
                # angle of 0 is pointing along the positive x-axis
                angle = angle_iter * math.pi / self.samples_per_orbit
                
                grasp = moveit_msgs.msg.Grasp()
                grasp.id = "cylindrical"
                
                # open and closed joint angles
                grasp.pre_grasp_posture = self.generate_hand_posture(
                        self.joint_names, self.cylindrical_open)
                grasp.grasp_posture = self.generate_hand_posture(
                        self.joint_names, self.cylindrical_closed)
                
                # pre-grasp approach description
                grasp.pre_grasp_approach = self.generate_gripper_translation(
                        self.gripper_link, [0.0, 0.0, 1.0], pregrasp_distance)
                
                # lift the object after grasping
                grasp.post_grasp_retreat = self.generate_gripper_translation(
                        "base_link", [0.0, 0.0, 1.0], pregrasp_distance)
                
                # after releasing the object, go to the post-grasp pose
                grasp.post_place_retreat = self.generate_gripper_translation(
                        self.gripper_link, [0.0, 0.0, -1.0], pregrasp_distance)
                
                wrist_pose = self.generate_side_grasp_matrix(
                        angle, grasp_distance, wrist_roll)
                grasp.grasp_pose = self.matrix_to_pose("object_link",
                        wrist_pose)
                
                grasps.append(grasp)
        return grasps


    def generate_grasps_from_top(self, grasp_distance, pregrasp_distance):
        '''
        :param grasp_distance: The distance from the object to the wrist's
        coordinate system in the final grasp.
        :type distance: float
        
        :param pregrasp_distance: The distance from the object to the wrist's
        coordinate system for the pre-grasp.
        :type distance: float
        
        :return: A list of generated grasps
        :rtype: moveit_msgs.msg.Grasp[]
        '''
        grasps = []
        for angle_iter in range(0, 2 * int(self.samples_per_orbit)):
            # angle is measured about z-axis in base link
            # angle of 0 is pointing along the positive x-axis
            angle = angle_iter * math.pi / self.samples_per_orbit
            
            grasp = moveit_msgs.msg.Grasp()
            grasp.id = "spherical"
            
            # open and closed joint angles
            grasp.pre_grasp_posture = self.generate_hand_posture(
                    self.joint_names, self.spherical_open)
            grasp.grasp_posture = self.generate_hand_posture(
                    self.joint_names, self.spherical_closed)
            
            # pre-grasp approach description
            grasp.pre_grasp_approach = self.generate_gripper_translation(
                    self.gripper_link, [0.0, 0.0, 1.0], pregrasp_distance)
            
            # lift the object after grasping
            grasp.post_grasp_retreat = self.generate_gripper_translation(
                    "base_link", [0.0, 0.0, 1.0], pregrasp_distance)
            
            # after releasing the object, go to the post-grasp pose
            grasp.post_place_retreat = self.generate_gripper_translation(
                    self.gripper_link, [0.0, 0.0, -1.0], pregrasp_distance)
            
            wrist_pose = self.generate_top_grasp_matrix(angle, grasp_distance)
            grasp.grasp_pose = self.matrix_to_pose("object_link",
                    wrist_pose)
            
            grasps.append(grasp)
        return grasps


    def generate_side_grasp_matrix(self, angle, distance, wrist_roll):
        '''
        The object's reference frame is assumed to be aligned with the robot's
        base frame, but offset by a translation. This method will generate a
        pose on a circular orbit around the object's reference frame. The orbit
        is aligned with the x-y-plane of the object.
        
        :param angle: The angle (in radians) about the z-axis of the object's
        reference coordinate frame.
        :type angle: float
        
        :param distance: The radius (in meters) of the the orbit, i.e. the
        distance of the hand's reference frame to the object's reference frame.
        :type angle: float
        
        :param wrist_roll: The roll (in radians) of the wrist w.r.t. the hand's
        local coordinate frame.
        :type wrist_roll: float
        
        :return: The 4x4 translation matrix representing the wrist's pose.
        :rtype: numpy.matrix
        '''
        wrist_orientation = numpy.matrix([
                            [0.0,  math.sin(angle), math.cos(angle), 0.0],
                            [0.0, -math.cos(angle), math.sin(angle), 0.0],
                            [1.0,  0.0,  0.0, 0.0],
                            [0.0,  0.0,  0.0, 1.0]])
        wrist_translation = numpy.matrix([
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, -distance],
                            [0.0, 0.0, 0.0, 1.0]])
        rot_z = numpy.matrix([
                [math.cos(wrist_roll), -math.sin(wrist_roll), 0.0, 0.0],
                [math.sin(wrist_roll), math.cos(wrist_roll), 0.0, 0.0],
                [0.0,  0.0, 1.0, 0.0],
                [0.0,  0.0, 0.0, 1.0]])
        
        return wrist_orientation * wrist_translation * rot_z


    def generate_top_grasp_matrix(self, angle, distance):
        '''
        The object's reference frame is assumed to be aligned with the robot's
        base frame, but offset by a translation. This method will generate a
        pose facing down (i.e. along the negative z-axis of the object's
        coordinate system).
        
        :param angle: The angle (in radians) about the z-axis of the object's
        reference coordinate frame.
        :type angle: float
        
        :param distance: The distance (in meters) from the hand's coordinate
        system to the object's coordinate system.
        :type distance: float
        
        :return: The 4x4 translation matrix representing the wrist's pose.
        :rtype: numpy.matrix
        '''
        wrist_orientation = numpy.matrix([
                            [-math.sin(angle), math.cos(angle), 0.0, 0.0],
                            [ math.cos(angle), math.sin(angle), 0.0, 0.0],
                            [ 0.0,  0.0, -1.0, 0.0],
                            [ 0.0,  0.0, 0.0, 1.0]])
        wrist_translation = numpy.matrix([
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, -distance],
                            [0.0, 0.0, 0.0, 1.0]])
        
        return wrist_orientation * wrist_translation


    def generate_gripper_translation(self, frame, vector, distance):
        '''
        :param frame: Name of the reference frame in which the gripper
        translation is specified.
        :type frame: String
        
        :param vector: The translation of the gripper as three-dimensional
        vector.
        :type vector: Float[3]
        
        :param distance: The distance that the hand should be translated along
        the provided vector.
        :type distance: Float
        
        :return: A structure describing the gripper translation
        :rtype: moveit_msgs.msg.GripperTranslation
        '''
        translation = moveit_msgs.msg.GripperTranslation()
        
        translation.direction.header.frame_id = frame
        translation.direction.header.stamp = rospy.Time.now()
        translation.direction.vector.x = vector[0]
        translation.direction.vector.y = vector[1]
        translation.direction.vector.z = vector[2]
        translation.desired_distance = distance
        translation.min_distance = distance * 0.75
        
        return translation


    def matrix_to_pose(self, frame, matrix):
        '''
        :param frame: Name of the reference frame in which the pose is
        specified.
        :type frame: String
        
        :param matrix: The 4x4 transformation matrix.
        :type matrix: numpy.matrix
        
        :return: The pose interpretable by ROS.
        :rtype: geometry_msgs.msg.PoseStamepd
        '''
        pose = geometry_msgs.msg.PoseStamped()
        
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]
        
        quat = tf.transformations.quaternion_from_matrix(matrix)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose


    def generate_hand_posture(self, joint_names, configuration):
        '''
        :param joint_names: The name of the joints that make up the hand's
        posture. The size of this list must match the size of the configuration
        list.
        :type joint_names: String[]
        
        :param configuration: The joint configuration of the hand's posture. The
        size of this list must match the size of the joint_names list.
        :type configuration: Float[]
        
        :return: The trajectory for the specified fingers.
        :rtype: trajectory_msgs.msg.JointTrajectory
        '''
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = configuration
        point.time_from_start = rospy.Duration(3.0)
        
        posture = trajectory_msgs.msg.JointTrajectory()
        posture.joint_names = joint_names
        posture.points.append(point)
        
        return posture