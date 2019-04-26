import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Twist
from nav_msgs.msg import Path
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ros_dmp.srv import GenerateMotion, GenerateMotionRequest

from mdr_move_arm_action.roll_dmp import RollDMP

class DMPExecutor(object):
    def __init__(self, dmp_name, tau):
        self.tf_listener = tf.TransformListener()
        self.base_link_frame_name = rospy.get_param('~base_link_frame_name', '/base_link')
        self.odom_frame_name = rospy.get_param('~odom_frame_name', '/odom')
        self.map_frame_name = rospy.get_param('~map_frame_name', '/map')
        self.palm_link_name = rospy.get_param('~palm_link_name', '/palm_link')
        self.cartesian_velocity_topic = rospy.get_param('~cartesian_velocity_topic',
                                                        '/arm_controller/cartesian_velocity_command')
        self.base_vel_topic = rospy.get_param('~base_vel_topic', '/cmd_vel')
        self.arm_controller_sigma_values_topic = rospy.get_param('~arm_controller_sigma_values_topic',
                                                                 '/arm_controller/sigma_values')
        self.dmp_executor_path_topic = rospy.get_param('~path_topic', '/dmp_executor/path')
        self.move_base_server = rospy.get_param('~move_base_server', 'move_base/move')

        # ros_dmp service
        self.motion_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)

        self.number_of_sampling_points = 30
        self.goal_tolerance = 0.05
        self.vel_publisher_arm = rospy.Publisher(self.cartesian_velocity_topic,
                                                 TwistStamped, queue_size=1)
        self.vel_publisher_base = rospy.Publisher(self.base_vel_topic, Twist, queue_size=1)
        self.feedforward_gain = 30
        self.feedback_gain = 10
        self.sigma_threshold_upper = 0.12
        self.sigma_threshold_lower = 0.07
        self.base_feedback_gain = 2.0

        rospy.Subscriber(self.arm_controller_sigma_values_topic,
                         Float32MultiArray, self.sigma_values_cb)
        self.path_pub = rospy.Publisher(self.dmp_executor_path_topic, Path, queue_size=1)
        self.goal = None
        self.dmp_name = dmp_name
        self.tau = tau

        self.min_sigma_value = None
        self.deploy_wbc = True

        # Move base server
        self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
        self.move_base_client.wait_for_server()

    def sigma_values_cb(self, msg):
        self.min_sigma_value = min(msg.data)

    def move_base(self):
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = self.map_frame_name
        print(self.move_base_client.send_goal(move_base_goal))

    def generate_trajectory(self, goal, initial_pos):
        req = GenerateMotionRequest()
        req.goal_pose.pose.position.x = goal[0]
        req.goal_pose.pose.position.y = goal[1]
        req.goal_pose.pose.position.z = goal[2]
        req.initial_pose.pose.position.x = initial_pos[0]
        req.initial_pose.pose.position.y = initial_pos[1]
        req.initial_pose.pose.position.z = initial_pos[2]
        req.dmp_name = self.dmp_name
        req.tau = self.tau
        req.dt = 0.001

        print('[dmp] Sending trajectory request: ', req)
        response = self.motion_client(req)
        pos_x = []
        pos_y = []
        pos_z = []
        for state in response.cart_traj.cartesian_state:
            pos_x.append(state.pose.position.x)
            pos_y.append(state.pose.position.y)
            pos_z.append(state.pose.position.z)
        self.pos = np.array(pos_x)
        self.pos = np.vstack((self.pos, np.array(pos_y)))
        self.pos = np.vstack((self.pos, np.array(pos_z)))

    def tranform_pose(self, pose):
        #transform goals to odom frame
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.base_link_frame_name
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]

        while not rospy.is_shutdown():
            try:
                pose_ = self.tf_listener.transformPose(self.odom_frame_name, pose_msg)
                break
            except:
                continue
        return np.array([pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z])

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.odom_frame_name
        for itr in range(self.pos.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = self.pos[itr, 0]
            pose_stamped.pose.position.y = self.pos[itr, 1]
            pose_stamped.pose.position.z = self.pos[itr, 2]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)

    def trajectory_controller(self):
        count = 0
        previous_index = 0
        path = self.pos
        path_x = path[0, :]
        path_y = path[1, :]
        path_z = path[2, :]
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame_name,
                                                                self.palm_link_name,
                                                                rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        current_pos = np.array([trans[0], trans[1], trans[2]])
        previous_pos = current_pos[:]
        distance = np.linalg.norm((np.array(path[:, path.shape[1] - 1]) - current_pos))
        followed_trajectory = []

        old_pos_index = 0
        while distance > self.goal_tolerance and not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame_name,
                                                                self.palm_link_name,
                                                                rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance = np.linalg.norm((np.array(path[:, path.shape[1] - 1]) - current_pos))
            dist = []
            for i in range(path.shape[1]):
                dist.append(np.linalg.norm((path[:, i] - current_pos)))
            index =  np.argmin(dist)

            if old_pos_index != index:
                followed_trajectory.append(current_pos)
                old_pos_index = index

            if index < previous_index:
                index = previous_index
            else:
                previous_index = index

            # Delete this block later
            if index > path.shape[1] - 1:
                break

            if index == path.shape[1] - 1:
                ind = index
            else:
                ind = index + 1

            vel_x = self.feedforward_gain * (path_x[ind] - path_x[index]) + self.feedback_gain * (path_x[ind] - current_pos[0])
            vel_y = self.feedforward_gain * (path_y[ind] - path_y[index]) + self.feedback_gain * (path_y[ind] - current_pos[1])
            vel_z = self.feedforward_gain * (path_z[ind] - path_z[index]) + self.feedback_gain * (path_z[ind] - current_pos[2])

            # limiting speed
            norm_ = np.linalg.norm(np.array([vel_x, vel_y, vel_z]))
            if norm_ > 0.05:
                vel_x = vel_x * 0.05 / norm_
                vel_y = vel_y * 0.05 / norm_
                vel_z = vel_z * 0.05 / norm_

            vel_x_arm = vel_x
            vel_y_arm = vel_y
            vel_z_arm = vel_z

            vel_x_base = 0.0
            vel_y_base = 0.0
            vel_z_base = 0.0
            ratio = 1.0

            if self.min_sigma_value != None and self.min_sigma_value < self.sigma_threshold_upper and self.deploy_wbc:
                ratio = (self.min_sigma_value - self.sigma_threshold_lower) / (self.sigma_threshold_upper - self.sigma_threshold_lower)

                vel_x_arm = vel_x * (ratio)
                vel_y_arm = vel_y * (ratio)
                vel_x_base = vel_x * (1 - ratio)
                vel_y_base = vel_y * (1 - ratio)

                # Publish base velocity inside the if consition
                vector_ = Vector3Stamped()
                vector_.header.seq = count
                vector_.header.frame_id = self.odom_frame_name
                vector_.vector.x = vel_x_base
                vector_.vector.y = vel_y_base
                vector_.vector.z = vel_z_base

                vector_ = self.tf_listener.transformVector3(self.base_link_frame_name, vector_)

                message_base = Twist()
                message_base.linear.x = vector_.vector.x
                message_base.linear.y = vector_.vector.y
                message_base.linear.z = vector_.vector.z
                self.vel_publisher_base.publish(message_base)

            message_arm = TwistStamped()
            message_arm.header.seq = count
            message_arm.header.frame_id = self.odom_frame_name
            message_arm.twist.linear.x = vel_x_arm
            message_arm.twist.linear.y = vel_y_arm
            message_arm.twist.linear.z = vel_z_arm
            self.vel_publisher_arm.publish(message_arm)
            count += 1

        # stop arm and base motion after converging
        message_base = Twist()
        message_base.linear.x = 0.0
        message_base.linear.y = 0.0
        message_base.linear.z = 0.0

        message_arm = TwistStamped()
        message_arm.header.seq = count
        message_arm.header.frame_id = self.odom_frame_name
        message_arm.twist.linear.x = 0
        message_arm.twist.linear.y = 0
        message_arm.twist.linear.z = 0

        self.vel_publisher_arm.publish(message_arm)
        if self.deploy_wbc:
            self.vel_publisher_base.publish(message_base)

    def execute(self, goal):
        initial_pos = None
        try:
            self.tf_listener.waitForTransform(self.base_link_frame_name,
                                              self.palm_link_name,
                                              rospy.Time.now(),
                                              rospy.Duration(30))
            (trans, rot) = self.tf_listener.lookupTransform(self.base_link_frame_name,
                                                            self.palm_link_name,
                                                            rospy.Time(0))
            initial_pos = np.array(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            initial_pos = np.zeros(3)

        self.generate_trajectory(goal, initial_pos)
        pos = []
        for i in range(self.pos.shape[1]):
            pos.append(self.tranform_pose(self.pos[:, i]))
        pos = np.array(pos)
        self.pos = pos.T
        self.publish_path()

        # transform pose to base link
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.odom_frame_name
        start_pose.pose.position.x = self.pos[0, 0]
        start_pose.pose.position.y = self.pos[0, 1]
        start_pose.pose.position.z = self.pos[0, 2]

        while not rospy.is_shutdown():
            try:
                start_pose = self.tf_listener.transformPose(self.base_link_frame_name, start_pose)
                break
            except:
                continue
        start_pose.pose.orientation.x = 0.529
        start_pose.pose.orientation.y = -0.475
        start_pose.pose.orientation.z = 0.467
        start_pose.pose.orientation.w = 0.525

        rospy.loginfo('Executing motion')
        self.trajectory_controller()
