#!/usr/bin/env python3
"""
CheckDoorOpen smach state.

Blocks and waits for the door_open_checker node to report that the door is open.
Returns 'open' as soon as the door is detected open, or 'closed' / 'timeout'
if the door never opens within the wait window.
"""

import rospy
import smach
from std_msgs.msg import Bool


class CheckDoorOpen(smach.State):
    """
    Wait for a door to be reported open by the door_open_checker node.

    Subscribes to a std_msgs/Bool topic published by the door_open_checker node
    and blocks until the door is open or the wait time expires.

    Parameters
    ----------
    result_topic : str
        The Bool topic published by door_open_checker (e.g. '/door_open_checker/result').
    wait_timeout : float
        How many seconds to wait for the door to open before giving up.
        Set to -1 to wait forever (default: 30.0).
    poll_rate : float
        Rate (Hz) at which to check the received door state (default: 10.0).

    Outcomes
    --------
    open    – door was detected open within wait_timeout
    closed  – door was never open and wait_timeout expired
    timeout – no message received on result_topic within wait_timeout
              (node not running / topic not publishing)
    """

    def __init__(self,
                 result_topic,
                 wait_timeout=30.0,
                 poll_rate=10.0):
        smach.State.__init__(
            self,
            outcomes=['open', 'closed', 'timeout']
        )
        self.result_topic = result_topic
        self.wait_timeout = wait_timeout
        self.poll_rate = poll_rate

        self._door_open = None   # None = no message received yet
        self._sub = rospy.Subscriber(
            self.result_topic, Bool, self._door_cb, queue_size=1
        )
        rospy.loginfo('[CheckDoorOpen] Subscribed to "%s"', self.result_topic)

    # ------------------------------------------------------------------ #

    def _door_cb(self, msg):
        self._door_open = msg.data

    # ------------------------------------------------------------------ #

    def execute(self, userdata):
        rospy.loginfo('[CheckDoorOpen] Waiting up to %.1fs for door to open on "%s" ...',
                      self.wait_timeout, self.result_topic)

        self._door_open = None  # reset for each execution
        rate = rospy.Rate(self.poll_rate)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()

            # Check for overall wait timeout
            if self.wait_timeout >= 0 and elapsed >= self.wait_timeout:
                if self._door_open is None:
                    rospy.logwarn(
                        '[CheckDoorOpen] No message received on "%s" after %.1fs',
                        self.result_topic, elapsed
                    )
                    return 'timeout'
                rospy.loginfo(
                    '[CheckDoorOpen] Wait timeout (%.1fs) reached – door was not opened.',
                    self.wait_timeout
                )
                return 'closed'

            if self._door_open is None:
                rospy.loginfo_throttle(
                    5.0, '[CheckDoorOpen] Waiting for first message on "%s" ...',
                    self.result_topic
                )
                rate.sleep()
                continue

            if self._door_open:
                rospy.loginfo('[CheckDoorOpen] Door is OPEN after %.1fs.', elapsed)
                return 'open'

            rospy.loginfo_throttle(
                2.0, '[CheckDoorOpen] Door still closed (elapsed: %.1fs)', elapsed
            )
            rate.sleep()

        # Interrupted by ROS shutdown
        return 'closed'
