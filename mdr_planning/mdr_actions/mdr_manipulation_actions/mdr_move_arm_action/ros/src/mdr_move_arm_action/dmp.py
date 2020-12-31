import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Twist
from nav_msgs.msg import Path
import tf
from ros_dmp.roll_dmp import RollDmp
import pydmps
from mas_tools.file_utils import load_yaml_file

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

        self.vel_publisher_arm = rospy.Publisher(self.cartesian_velocity_topic,
                                                 TwistStamped, queue_size=1)
        self.vel_publisher_base = rospy.Publisher(self.base_vel_topic, Twist, queue_size=1)

        self.time_step = rospy.get_param('~time_step', 0.001)
        self.goal_tolerance = rospy.get_param('~goal_tolerance_m', 0.05)
        self.feedforward_gain = rospy.get_param('~feedforward_gain', 30)
        self.feedback_gain = rospy.get_param('~feedback_gain', 10)
        self.sigma_threshold_upper = rospy.get_param('~sigma_threshold_upper', 0.12)
        self.sigma_threshold_lower = rospy.get_param('~sigma_threshold_lower', 0.07)
        self.use_whole_body_control = rospy.get_param('~use_whole_body_control', False)
        self.linear_vel_limit = rospy.get_param('~linear_vel_limit', 0.05)

        rospy.Subscriber(self.arm_controller_sigma_values_topic,
                         Float32MultiArray, self.sigma_values_cb)
        self.path_pub = rospy.Publisher(self.dmp_executor_path_topic, Path, queue_size=1)

        self.dmp_name = dmp_name
        self.tau = tau
        self.roll_dmp = RollDmp(self.dmp_name, self.time_step, self.base_link_frame_name)

        self.min_sigma_value = None
        self.goal = None
        self.motion_completed = False
        self.motion_cancelled = False

    def move_to(self, goal_pose):
        '''Moves the end effector to the given goal position (which is expected
        to be given with respect to the base link frame) by following a trajectory
        as encoded by self.dmp_name.

        Keyword arguments:
        goal_pose: geometry_msgs.msg.PoseStamped -- end effector goal pose

        '''
        (trans, _) = self.get_transform(self.odom_frame_name, self.palm_link_name, rospy.Time.now())
        initial_pos_odom = np.array(trans)
        goal_odom = self.transform_pose(goal_pose, self.odom_frame_name)
        self.follow_path(initial_pos_odom, goal_odom)

    def follow_path(self, initial_pos_odom,  goal_pose_odom):
        '''Moves a manipulator so that it follows the given path. If whole body
        motion is enabled and some points on the path lie outside the reachable
        workspace, the base is moved accordingly as well. The path is followed
        by sending velocity commands.

        At each time step, the following rule is used for calculating the
        desired velocity command:

        v = k * (path_{i+1} - path_{i}) + g * (path_{i} - p)

        where:
        * v is the velocity (x, y, and z - no angular velocity)
        * p is the current position of the end effector
        * g is a feedback gain
        * k is a feedforward gain

        If whole body motion is used and the minimum singular value of the manipulator's
        Jacobian falls below a predefined threshold, v is split between the arm and the base as

        v_{arm} = v * c
        v_{base} = v * (1 - c)

        where:

        * c = (\sigma_{min} - \sigma_{low}) / (\sigma{high} - \sigma_{low})
        * \sigma_{min} is the current minimum singular value of the manipulator's Jacobian
        * \sigma_{high} and \sigma_{low} are upper and lower thresholds on the
          minimum sigma value (determined experimentally)

        Keyword arguments:
        initial_pos_odom: numpy.ndarray -- the initial position of the end effector
                                           in the odometry frame
        goal_pose_odom: geometry_msgs.msg.PoseStamped -- end effector goal pose
                                                         in the odometry frame

        '''
        rospy.loginfo('[move_arm/dmp/follow_path] Executing motion')
        trans, _ = self.get_transform(self.odom_frame_name, self.palm_link_name, rospy.Time(0))
        current_pos = np.array([trans[0], trans[1], trans[2]])
        goal = np.array([goal_pose_odom.pose.position.x,
                         goal_pose_odom.pose.position.y,
                         goal_pose_odom.pose.position.z])

        distance_to_goal = np.linalg.norm((goal - current_pos))

        initial_pose = np.array([initial_pos_odom[0],
                                 initial_pos_odom[1],
                                 initial_pos_odom[2],
                                 0., 0., 0.])
        goal_pose = np.array([goal[0], goal[1], goal[2], 0., 0., 0.])

        current_path = initial_pose[:3]

        self.dmp = self.instantiate_dmp(initial_pose, goal_pose)

        self.motion_completed = False
        self.motion_cancelled = False
        cmd_count = 0
        while not self.motion_completed and \
              not self.motion_cancelled and \
              not rospy.is_shutdown():

            trans, _ = self.get_transform(self.odom_frame_name, self.palm_link_name, rospy.Time(0))
            current_pos = np.array([trans[0], trans[1], trans[2]])

            # if the end effector has reached the goal (within the allowed
            # tolerance threshold), we stop the motion
            distance_to_goal = np.linalg.norm((goal - current_pos))
            if distance_to_goal <= self.goal_tolerance:
                self.motion_completed = True
                break

            next_pose, _, _ = self.dmp.step(tau=self.tau)
            next_pos = next_pose[:3]
            current_path = np.vstack((current_path, next_pos))

            vel = self.feedback_gain * (next_pos - current_pos)

            # we limit the speed if it is above the allowed limit
            velocity_norm = np.linalg.norm(vel)
            if velocity_norm > self.linear_vel_limit:
                vel = vel * self.linear_vel_limit / velocity_norm

            vel_arm = np.array(vel)
            vel_base = np.zeros(3)

            # if we want to use whole body control and the minimum
            # sigma value is below the allowed threshold, we split
            # the velocity command between the arm and the base
            if self.use_whole_body_control and \
               self.min_sigma_value is not None and \
               self.min_sigma_value < self.sigma_threshold_upper:

                # we set the arm and base velocity based on the value of the
                # so-called capability coefficient, which is calculated as
                #     (\sigma_{min} - \sigma_{low}) / (\sigma{high} - \sigma_{low})
                c = (self.min_sigma_value - self.sigma_threshold_lower) / (self.sigma_threshold_upper - self.sigma_threshold_lower)
                vel_arm[0:2] = vel[0:2] * c
                vel_base[0:2] = vel[0:2] * (1 - c)

                odom_vel_vector = Vector3Stamped()
                odom_vel_vector.header.seq = cmd_count
                odom_vel_vector.header.frame_id = self.odom_frame_name
                odom_vel_vector.vector.x = vel_base[0]
                odom_vel_vector.vector.y = vel_base[1]
                odom_vel_vector.vector.z = vel_base[2]
                base_vel_vector = self.tf_listener.transformVector3(self.base_link_frame_name, odom_vel_vector)

                twist_base = Twist()
                twist_base.linear.x = base_vel_vector.vector.x
                twist_base.linear.y = base_vel_vector.vector.y
                twist_base.linear.z = base_vel_vector.vector.z
                self.vel_publisher_base.publish(twist_base)

            twist_arm = TwistStamped()
            twist_arm.header.seq = cmd_count
            twist_arm.header.frame_id = self.odom_frame_name
            twist_arm.twist.linear.x = vel_arm[0]
            twist_arm.twist.linear.y = vel_arm[1]
            twist_arm.twist.linear.z = vel_arm[2]
            self.vel_publisher_arm.publish(twist_arm)
            cmd_count += 1

            self.publish_path(current_path)

        # stop arm and base motion after converging
        twist_arm = TwistStamped()
        twist_arm.header.seq = cmd_count
        twist_arm.header.frame_id = self.odom_frame_name
        self.vel_publisher_arm.publish(twist_arm)

        twist_base = Twist()
        if self.use_whole_body_control:
            self.vel_publisher_base.publish(twist_base)

    def get_transform(self, target_frame, source_frame, tf_time):
        '''Returns the translation and rotation of the source frame
        with respect to the target frame at the given time.

        Keyword arguments:
        target_frame: str -- name of the transformation target frame
        source_frame: str -- name of the transformation source frame
        tf_time: rospy.rostime.Time -- time of the transform

        '''
        trans = None
        rot = None
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, tf_time)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return (trans, rot)

    def transform_pose(self, pose_msg, target_frame):
        '''Transforms the given pose to the target frame.

        Keyword arguments:
        pose_msg: geometry_msgs.msg.PoseStamped -- pose to be transformed
        target_frame: str -- name of the transformation target frame

        '''
        transformed_pose = pose_msg
        while not rospy.is_shutdown():
            try:
                transformed_pose = self.tf_listener.transformPose(target_frame, pose_msg)
                break
            except:
                continue
        return transformed_pose

    def publish_path(self, path):
        '''Publishes the given path to the topic specified by self.dmp_executor_path_topic.

        Keyword arguments:
        path: numpy.ndarray -- a 2D array of points in which each row represents a position

        '''
        path_msg = Path()
        path_msg.header.frame_id = self.odom_frame_name
        for i in range(path.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = path[i,0]
            pose_stamped.pose.position.y = path[i,1]
            pose_stamped.pose.position.z = path[i,2]
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

    def sigma_values_cb(self, msg):
        '''Callback function for registering the singular values
        of a manipulator's Jacobian. Saves the lowest value given
        in the input message in self.min_sigma_value.

        Keyword arguments:
        msg: std_msgs.msg.Float32MultiArray

        '''
        self.min_sigma_value = min(msg.data)

    def instantiate_dmp(self, initial_pose, goal_pose):
        '''Instantiates a DMP object from learned weights, given
        the initial and final poses.

        Keyword arguments:
        initial_pose: numpy.ndarray -- initial pose coordinates of end-effector
        goal_pose: numpy.ndarray -- target pose coordinates of end-effector
        '''
        dmp_weights_dict = load_yaml_file(self.dmp_name)
        n_dmps, n_bfs = len(dmp_weights_dict), len(dmp_weights_dict['x'])

        dmp_weights = np.zeros((n_dmps, n_bfs))
        dmp_weights[0, :] = dmp_weights_dict['x']
        dmp_weights[1, :] = dmp_weights_dict['y']
        dmp_weights[2, :] = dmp_weights_dict['z']
        dmp_weights[3, :] = dmp_weights_dict['roll']
        dmp_weights[4, :] = dmp_weights_dict['pitch']
        dmp_weights[5, :] = dmp_weights_dict['yaw']

        return pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps,
                                                 n_bfs=n_bfs,
                                                 dt=self.time_step,
                                                 y0=initial_pose,
                                                 goal=goal_pose,
                                                 ay=None,
                                                 w=dmp_weights)
