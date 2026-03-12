#!/usr/bin/env python3
"""
Robot Motion States - HSR joint/head/gripper control for pick-and-place tasks.

Tunable joint values are at the top of this file.
Reference: /home/lucy/ros/noetic/src/notebook (robot control notebooks)

HSR joint limits (rad unless noted):
  arm_lift_joint    : 0.0 – 0.69 m   (0 = lowest, 0.69 = highest)
  head_tilt_joint   : -0.62 (up) – 0.52 (down)
  head_pan_joint    : -3.84 – 1.75
  wrist_flex_joint  : -1.92 – 1.22
  hand_motor_joint  : -0.79 (closed) – 1.24 (open)
"""

import math
import numpy as np
import rospy
import smach
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Tunable defaults — adjust after checking on the robot
# ---------------------------------------------------------------------------

# Lift heights (metres)
LIFT_NEUTRAL   = 0.0    # lowest position (for table-level tasks)
LIFT_SHELF     = 0.25   # raised position for perceiving shelf / side table
LIFT_BIN_DROP  = 0.20   # height when dropping into bin

# Head angles (radians)
HEAD_TILT_NEUTRAL  =  0.0    # straight ahead
HEAD_TILT_TABLE    =  0.20   # slight down — looking at table surface
HEAD_TILT_BIN      =  0.45   # looking down at floor-level bin
HEAD_TILT_SHELF    = -0.20   # slight up — looking at raised shelf
HEAD_PAN_NEUTRAL   =  0.0

# Arm pose for dropping object over bin (degrees → rad already applied)
# Arm extended forward-down, wrist pointing down
DROP_ARM_JOINTS = {
    # joint_name: position (rad or m)
    'arm_lift_joint':   LIFT_BIN_DROP,
    'arm_flex_joint':  -0.70,   # flex toward front
    'arm_roll_joint':   0.0,
    'wrist_flex_joint': -1.57,  # wrist pointing straight down
    'wrist_roll_joint': 0.0,
}

# Gripper
GRIPPER_OPEN   =  1.20   # fully open
GRIPPER_CLOSED = -0.79   # fully closed

# Topic names (HSR defaults)
HEAD_TRAJ_TOPIC    = '/hsrb/head_trajectory_controller/command'
ARM_TRAJ_TOPIC     = '/hsrb/arm_trajectory_controller/command'
GRIPPER_TRAJ_TOPIC = '/hsrb/gripper_controller/command'
BASE_VEL_TOPIC     = '/hsrb/command_velocity'

# Arm pose for reaching into washing machine (from go_to_cleaning_pose.py notebook)
CLEANING_ARM_JOINTS = {
    'arm_flex_joint':   0.0004,   # ~0 (straight)
    'arm_lift_joint':   0.0,      # lowest
    'arm_roll_joint':  -1.5700,   # rolled to side
    'wrist_flex_joint':-1.5700,   # wrist pointing forward/down
    'wrist_roll_joint': 0.0,
}

# Safe carry/neutral pose for navigating with clothes
NEUTRAL_ARM_JOINTS = {
    'arm_lift_joint':   0.0,
    'arm_flex_joint':  -0.70,
    'arm_roll_joint':  -1.57,
    'wrist_flex_joint':-1.57,
    'wrist_roll_joint': 0.0,
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_traj(joint_names, positions, duration=1.5):
    """Build a JointTrajectory with a single waypoint."""
    traj = JointTrajectory()
    traj.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    return traj


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class SetHeadPose(smach.State):
    """
    Set head pan and tilt angles.

    Params (all optional):
        pan   (float) - pan angle in radians  (default: HEAD_PAN_NEUTRAL)
        tilt  (float) - tilt angle in radians (default: HEAD_TILT_NEUTRAL)
        duration (float) - seconds to wait for motion (default: 1.5)

    Outcomes: succeeded
    """

    def __init__(self,
                 pan=HEAD_PAN_NEUTRAL,
                 tilt=HEAD_TILT_NEUTRAL,
                 duration=1.5):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pan = pan
        self.tilt = tilt
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        if self._pub is None:
            self._pub = rospy.Publisher(HEAD_TRAJ_TOPIC, JointTrajectory,
                                        queue_size=1, latch=True)
            rospy.sleep(0.3)

        rospy.loginfo('[SetHeadPose] pan=%.2f  tilt=%.2f', self.pan, self.tilt)
        traj = _make_traj(
            ['head_pan_joint', 'head_tilt_joint'],
            [self.pan, self.tilt],
            duration=self.duration
        )
        self._pub.publish(traj)
        rospy.sleep(self.duration + 0.2)
        return 'succeeded'


class SetLiftJoint(smach.State):
    """
    Set the arm_lift_joint height (metres).

    Params:
        height   (float) - target height in metres (0.0 – 0.69)
        duration (float) - seconds to complete motion (default: 2.0)

    Outcomes: succeeded
    """

    def __init__(self, height=LIFT_NEUTRAL, duration=2.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.height = max(0.0, min(height, 0.69))
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        if self._pub is None:
            self._pub = rospy.Publisher(ARM_TRAJ_TOPIC, JointTrajectory,
                                        queue_size=1, latch=True)
            rospy.sleep(0.3)

        rospy.loginfo('[SetLiftJoint] height=%.3f m', self.height)
        traj = _make_traj(
            ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
             'wrist_flex_joint', 'wrist_roll_joint'],
            [self.height, 0.0, -1.57, -1.57, 0.0],  # neutral arm, raised torso
            duration=self.duration
        )
        self._pub.publish(traj)
        rospy.sleep(self.duration + 0.2)
        return 'succeeded'


class OpenGripper(smach.State):
    """
    Open the gripper fully.

    Outcomes: succeeded
    """

    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        if self._pub is None:
            self._pub = rospy.Publisher(GRIPPER_TRAJ_TOPIC, JointTrajectory,
                                        queue_size=1, latch=True)
            rospy.sleep(0.3)

        rospy.loginfo('[OpenGripper] Opening gripper')
        traj = _make_traj(
            ['hand_motor_joint'],
            [GRIPPER_OPEN],
            duration=self.duration
        )
        self._pub.publish(traj)
        rospy.sleep(self.duration + 0.2)
        return 'succeeded'


class DropInBin(smach.State):
    """
    Drop a held object into the bin.

    Flow:
        1. Raise lift joint to BIN_DROP height
        2. Extend arm forward-down over the bin (DROP_ARM_JOINTS pose)
        3. Open gripper to release object
        4. Retract arm to neutral

    Assumes the robot is already positioned at the bin (NavigateTo dust_bin done).

    Outcomes: succeeded | failed
    """

    def __init__(self, duration=2.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.duration = duration
        self._arm_pub = None
        self._gripper_pub = None

    def _get_arm_pub(self):
        if self._arm_pub is None:
            self._arm_pub = rospy.Publisher(ARM_TRAJ_TOPIC, JointTrajectory,
                                            queue_size=1, latch=True)
            rospy.sleep(0.3)
        return self._arm_pub

    def _get_gripper_pub(self):
        if self._gripper_pub is None:
            self._gripper_pub = rospy.Publisher(GRIPPER_TRAJ_TOPIC, JointTrajectory,
                                                queue_size=1, latch=True)
            rospy.sleep(0.3)
        return self._gripper_pub

    def execute(self, userdata):
        try:
            arm_pub = self._get_arm_pub()
            gripper_pub = self._get_gripper_pub()

            # 1. Move to drop pose (arm over bin)
            rospy.loginfo('[DropInBin] Moving arm to drop pose')
            names = list(DROP_ARM_JOINTS.keys())
            positions = [DROP_ARM_JOINTS[n] for n in names]
            arm_pub.publish(_make_traj(names, positions, duration=self.duration))
            rospy.sleep(self.duration + 0.3)

            # 2. Open gripper — drop object
            rospy.loginfo('[DropInBin] Opening gripper')
            gripper_pub.publish(_make_traj(
                ['hand_motor_joint'], [GRIPPER_OPEN], duration=1.0
            ))
            rospy.sleep(1.5)

            # 3. Retract arm to neutral carry pose
            rospy.loginfo('[DropInBin] Retracting arm')
            arm_pub.publish(_make_traj(
                ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                 'wrist_flex_joint', 'wrist_roll_joint'],
                [0.0, 0.0, -1.57, -1.57, 0.0],
                duration=self.duration
            ))
            rospy.sleep(self.duration + 0.2)

            return 'succeeded'

        except Exception as e:
            rospy.logerr('[DropInBin] Error: %s', e)
            return 'failed'


class ArmCleaningPose(smach.State):
    """
    Move arm to the washing machine reaching pose (from go_to_cleaning_pose.py notebook).
    Also sets head to neutral (looking forward).

    Outcomes: succeeded | failed
    """

    def __init__(self, duration=3.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.duration = duration
        self._arm_pub = None
        self._head_pub = None

    def execute(self, userdata):
        try:
            if self._arm_pub is None:
                self._arm_pub = rospy.Publisher(ARM_TRAJ_TOPIC, JointTrajectory,
                                                queue_size=1, latch=True)
                rospy.sleep(0.3)
            if self._head_pub is None:
                self._head_pub = rospy.Publisher(HEAD_TRAJ_TOPIC, JointTrajectory,
                                                 queue_size=1, latch=True)
                rospy.sleep(0.3)

            rospy.loginfo('[ArmCleaningPose] Moving arm to cleaning pose')
            names = list(CLEANING_ARM_JOINTS.keys())
            positions = [CLEANING_ARM_JOINTS[n] for n in names]
            self._arm_pub.publish(_make_traj(names, positions, duration=self.duration))
            rospy.sleep(self.duration + 0.3)

            rospy.loginfo('[ArmCleaningPose] Setting head neutral')
            self._head_pub.publish(_make_traj(
                ['head_pan_joint', 'head_tilt_joint'], [0.0, 0.0], duration=2.0
            ))
            rospy.sleep(2.2)

            return 'succeeded'
        except Exception as e:
            rospy.logerr('[ArmCleaningPose] Error: %s', e)
            return 'failed'


class CloseGripper(smach.State):
    """
    Close the gripper to grab laundry/objects.

    Params:
        position (float) - gripper joint target. Default: GRIPPER_CLOSED (-0.79 = fully closed).
                           Use a less negative value (e.g. -0.3) for a softer grip.
        duration (float) - seconds for motion (default: 2.0)

    Outcomes: succeeded
    """

    def __init__(self, position=GRIPPER_CLOSED, duration=2.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.position = position
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        if self._pub is None:
            self._pub = rospy.Publisher(GRIPPER_TRAJ_TOPIC, JointTrajectory,
                                        queue_size=1, latch=True)
            rospy.sleep(0.3)

        rospy.loginfo('[CloseGripper] Closing gripper to %.2f', self.position)
        self._pub.publish(_make_traj(
            ['hand_motor_joint'], [self.position], duration=self.duration
        ))
        rospy.sleep(self.duration + 0.3)
        return 'succeeded'


class MoveBaseVel(smach.State):
    """
    Move the robot base at a fixed velocity for a set duration.

    Params:
        vx       (float) - forward/backward velocity m/s (positive=forward, negative=backward)
        vy       (float) - lateral velocity m/s (default 0.0)
        duration (float) - seconds to apply velocity (default 2.0)

    Outcomes: succeeded
    """

    def __init__(self, vx=0.1, vy=0.0, duration=2.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.vx = vx
        self.vy = vy
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        if self._pub is None:
            self._pub = rospy.Publisher(BASE_VEL_TOPIC, Twist,
                                        queue_size=1)
            rospy.sleep(0.3)

        rospy.loginfo('[MoveBaseVel] vx=%.2f vy=%.2f for %.1fs', self.vx, self.vy, self.duration)
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy

        rate = rospy.Rate(10)
        start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start < self.duration:
            self._pub.publish(twist)
            rate.sleep()

        # Stop
        self._pub.publish(Twist())
        rospy.sleep(0.2)
        return 'succeeded'


class ArmNeutralPose(smach.State):
    """
    Move arm to safe neutral/carry pose for navigation.

    Outcomes: succeeded | failed
    """

    def __init__(self, duration=3.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.duration = duration
        self._pub = None

    def execute(self, userdata):
        try:
            if self._pub is None:
                self._pub = rospy.Publisher(ARM_TRAJ_TOPIC, JointTrajectory,
                                            queue_size=1, latch=True)
                rospy.sleep(0.3)

            rospy.loginfo('[ArmNeutralPose] Moving arm to neutral carry pose')
            names = list(NEUTRAL_ARM_JOINTS.keys())
            positions = [NEUTRAL_ARM_JOINTS[n] for n in names]
            self._pub.publish(_make_traj(names, positions, duration=self.duration))
            rospy.sleep(self.duration + 0.3)
            return 'succeeded'
        except Exception as e:
            rospy.logerr('[ArmNeutralPose] Error: %s', e)
            return 'failed'


class MoveToDistance(smach.State):
    """
    Move the robot forward until the laser scan measures a target distance
    to the nearest obstacle ahead, then stop.

    Params:
        stop_distance (float) - target gap to obstacle in metres (default 0.35)
        vx            (float) - forward speed in m/s (default 0.10)
        scan_topic    (str)   - laser scan topic (default /hsrb/base_scan)
        scan_angle    (float) - half-angle window in radians to average for
                                 "forward" distance (default 0.2 rad ~ ±11 deg)
        timeout       (float) - max seconds to drive before giving up (default 15.0)

    Outcomes: succeeded | failed (timeout or no laser data)
    """

    def __init__(self,
                 stop_distance=0.35,
                 vx=0.10,
                 scan_topic='/hsrb/base_scan',
                 scan_angle=0.2,
                 timeout=15.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.stop_distance = stop_distance
        self.vx = vx
        self.scan_topic = scan_topic
        self.scan_angle = scan_angle
        self.timeout = timeout
        self._vel_pub = None
        self._scan = None
        self._scan_sub = None

    def _scan_cb(self, msg):
        self._scan = msg

    def _get_forward_distance(self):
        """Return median distance of laser rays in the forward cone."""
        scan = self._scan
        if scan is None:
            return None
        n = len(scan.ranges)
        center = n // 2
        half = int(self.scan_angle / abs(scan.angle_increment)) if scan.angle_increment != 0 else 10
        lo = max(0, center - half)
        hi = min(n, center + half + 1)
        rays = [r for r in scan.ranges[lo:hi]
                if scan.range_min < r < scan.range_max]
        if not rays:
            return None
        return float(np.median(rays))

    def execute(self, userdata):
        if self._vel_pub is None:
            self._vel_pub = rospy.Publisher(BASE_VEL_TOPIC, Twist, queue_size=1)
            rospy.sleep(0.3)
        if self._scan_sub is None:
            self._scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb)
            rospy.sleep(0.5)  # let first scan arrive

        # Wait for first scan
        wait_start = rospy.Time.now().to_sec()
        while self._scan is None:
            if rospy.Time.now().to_sec() - wait_start > 3.0:
                rospy.logerr('[MoveToDistance] No laser data on %s', self.scan_topic)
                return 'failed'
            rospy.sleep(0.1)

        dist = self._get_forward_distance()
        if dist is None:
            rospy.logerr('[MoveToDistance] Cannot read forward distance')
            return 'failed'

        rospy.loginfo('[MoveToDistance] Current forward distance: %.3f m  target stop: %.3f m',
                      dist, self.stop_distance)

        if dist <= self.stop_distance:
            rospy.loginfo('[MoveToDistance] Already within stop distance, not moving')
            return 'succeeded'

        rate = rospy.Rate(10)
        start = rospy.Time.now().to_sec()
        twist = Twist()
        twist.linear.x = self.vx

        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start
            if elapsed > self.timeout:
                rospy.logwarn('[MoveToDistance] Timeout after %.1fs', elapsed)
                self._vel_pub.publish(Twist())
                return 'failed'

            dist = self._get_forward_distance()
            if dist is not None:
                rospy.loginfo_throttle(1.0, '[MoveToDistance] distance=%.3f m', dist)
                if dist <= self.stop_distance:
                    break

            self._vel_pub.publish(twist)
            rate.sleep()

        self._vel_pub.publish(Twist())  # stop
        rospy.sleep(0.2)
        rospy.loginfo('[MoveToDistance] Reached stop distance %.3f m', self.stop_distance)
        return 'succeeded'


class SetViewpointModePath(smach.State):
    """
    Set the HSR viewpoint controller to PATH mode so the head faces the direction
    the robot is travelling. Call once at SM startup.

    Outcomes: succeeded | failed
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/viewpoint_controller/set_viewpoint_mode_path', timeout=3.0)
            rospy.ServiceProxy('/viewpoint_controller/set_viewpoint_mode_path', Empty)()
            rospy.wait_for_service('/viewpoint_controller/start', timeout=3.0)
            rospy.ServiceProxy('/viewpoint_controller/start', Empty)()
            rospy.loginfo('[SetViewpointModePath] Viewpoint controller set to PATH mode')
            return 'succeeded'
        except Exception as e:
            rospy.logwarn('[SetViewpointModePath] Could not set viewpoint mode: %s', e)
            return 'failed'


class ViewpointControllerStop(smach.State):
    """
    Stop the HSR viewpoint controller so it does not move the head during navigation.
    Call this before NavigateTo to keep head facing forward (required for HRI scoring).

    Outcomes: succeeded | failed
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/viewpoint_controller/stop', timeout=3.0)
            stop = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
            stop()
            rospy.loginfo('[ViewpointControllerStop] Viewpoint controller stopped')
            return 'succeeded'
        except Exception as e:
            rospy.logwarn('[ViewpointControllerStop] Could not stop viewpoint controller: %s', e)
            return 'failed'


class ViewpointControllerStart(smach.State):
    """
    Re-enable the HSR viewpoint controller after navigation is done.

    Outcomes: succeeded | failed
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/viewpoint_controller/start', timeout=3.0)
            start = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
            start()
            rospy.loginfo('[ViewpointControllerStart] Viewpoint controller started')
            return 'succeeded'
        except Exception as e:
            rospy.logwarn('[ViewpointControllerStart] Could not start viewpoint controller: %s', e)
            return 'failed'
