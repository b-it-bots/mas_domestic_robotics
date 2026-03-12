#!/usr/bin/env python3
"""
IntroduceGuests + LocateSeatedGuests states.

LocateSeatedGuests: pans head left/right, uses ReID to find which guest
                    is seated on which side, stores result in userdata.

IntroduceGuests: reads guest JSON files and introduces each guest,
                 looking toward their seated side if known.
"""

import os
import json
import base64
import rospy
import smach

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

HEAD_TRAJ_TOPIC  = '/hsrb/head_trajectory_controller/command'
CAMERA_TOPIC     = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'

# Pan angles for looking at seated guests
PAN_LEFT     =  0.7   # ~40 deg left
PAN_RIGHT    = -0.7   # ~40 deg right
PAN_NEUTRAL  =  0.0
TILT_SEATED  =  0.10  # slight down — aimed at seated person's face


def _send_head(pan, tilt, duration=1.2):
    """Publish a single head trajectory command and wait for it."""
    pub = rospy.Publisher(HEAD_TRAJ_TOPIC, JointTrajectory, queue_size=1, latch=True)
    rospy.sleep(0.1)
    traj = JointTrajectory()
    traj.joint_names = ['head_pan_joint', 'head_tilt_joint']
    pt = JointTrajectoryPoint()
    pt.positions = [pan, tilt]
    pt.time_from_start = rospy.Duration(duration)
    traj.points = [pt]
    pub.publish(traj)
    rospy.sleep(duration + 0.3)


# ---------------------------------------------------------------------------

class LocateSeatedGuests(smach.State):
    """
    Pan the head left then right, capture an image at each side, and
    use the ReID service to identify which guest is seated where.

    Stores result in userdata.guest_positions:
        {'Alice': 'left', 'Bob': 'right'}   (or 'unknown_left' / 'unknown_right')

    Always returns 'succeeded' — recognition failure is non-fatal.

    Params:
        pan_left  (float) - pan angle for left scan  (default: 0.7 rad)
        pan_right (float) - pan angle for right scan (default: -0.7 rad)
        tilt      (float) - tilt while scanning (default: 0.10 rad)

    Output keys: guest_positions
    Outcomes: succeeded | failed
    """

    def __init__(self, pan_left=PAN_LEFT, pan_right=PAN_RIGHT, tilt=TILT_SEATED):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['guest_positions']
        )
        self.pan_left  = pan_left
        self.pan_right = pan_right
        self.tilt      = tilt
        self._bridge   = CvBridge() if CV_AVAILABLE else None
        self.vlm       = None

        try:
            from hsr_task_sm.srv import VLMQuery
            for svc in ['/reid/query', '/vlm/query']:
                try:
                    rospy.wait_for_service(svc, timeout=3.0)
                    self.vlm = rospy.ServiceProxy(svc, VLMQuery)
                    rospy.loginfo('[LocateSeatedGuests] Using %s', svc)
                    break
                except Exception:
                    continue
        except Exception as e:
            rospy.logwarn('[LocateSeatedGuests] No ReID service: %s', e)

    def _capture_and_recognize(self):
        """Capture one image and return recognized name or None."""
        if not CV_AVAILABLE or not self.vlm:
            return None
        try:
            msg = rospy.wait_for_message(CAMERA_TOPIC, RosImage, timeout=4.0)
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
            resp = self.vlm(query_type='recognize_person', image_base64=b64, context='')
            if resp.success:
                return resp.answer
        except Exception as e:
            rospy.logwarn('[LocateSeatedGuests] Capture/recognize error: %s', e)
        return None

    def execute(self, userdata):
        guest_positions = {}

        # --- Look left ---
        rospy.loginfo('[LocateSeatedGuests] Looking left...')
        _send_head(self.pan_left, self.tilt)
        rospy.sleep(0.5)
        left_name = self._capture_and_recognize()
        rospy.loginfo('[LocateSeatedGuests] Left side: %s', left_name or 'unknown')

        # --- Look right ---
        rospy.loginfo('[LocateSeatedGuests] Looking right...')
        _send_head(self.pan_right, self.tilt)
        rospy.sleep(0.5)
        right_name = self._capture_and_recognize()
        rospy.loginfo('[LocateSeatedGuests] Right side: %s', right_name or 'unknown')

        # --- Return to neutral ---
        _send_head(PAN_NEUTRAL, 0.0)

        # Build position map
        if left_name and left_name != 'unknown':
            guest_positions[left_name] = 'left'
        else:
            guest_positions['unknown_left'] = 'left'

        if right_name and right_name != 'unknown':
            guest_positions[right_name] = 'right'
        else:
            guest_positions['unknown_right'] = 'right'

        rospy.loginfo('[LocateSeatedGuests] Positions: %s', guest_positions)
        userdata.guest_positions = guest_positions
        return 'succeeded'


# ---------------------------------------------------------------------------

class IntroduceGuests(smach.State):
    """
    Introduce guests to each other by reading their saved info.

    Reads from guest JSON files and speaks introductions.
    If userdata.guest_positions is available, looks toward each guest
    before introducing them.

    Params:
        json_dir: directory containing guest JSON files
        retries:  max retries before failing

    Input keys:  guest_positions  (optional — dict {name: 'left'|'right'})
    Outcomes:    succeeded | failed | failed_after_retrying
    """

    def __init__(self, json_dir='/tmp/hri_guests', retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=['guest_positions']
        )
        self.json_dir    = json_dir
        self.retries     = retries
        self.retry_count = 0
        self.say_pub     = rospy.Publisher('/say', String, queue_size=10)

    def _say(self, text):
        rospy.loginfo('[IntroduceGuests] Saying: %s', text)
        self.say_pub.publish(String(data=text))
        num_words = len(text.split())
        rospy.sleep(max(1.0, num_words * 0.5))

    def _look_at(self, name, guest_positions):
        """Pan head toward the named guest's side, or neutral if unknown."""
        side = guest_positions.get(name) if guest_positions else None
        if side == 'left':
            rospy.loginfo('[IntroduceGuests] Looking left for %s', name)
            _send_head(PAN_LEFT, TILT_SEATED)
        elif side == 'right':
            rospy.loginfo('[IntroduceGuests] Looking right for %s', name)
            _send_head(PAN_RIGHT, TILT_SEATED)
        else:
            rospy.loginfo('[IntroduceGuests] Side unknown for %s — looking forward', name)
            _send_head(PAN_NEUTRAL, 0.0)

    def execute(self, userdata):
        rospy.loginfo('[IntroduceGuests] Starting introductions...')

        guest_positions = getattr(userdata, 'guest_positions', None) or {}

        # Load guest info
        json1 = os.path.join(self.json_dir, 'person1.json')
        json2 = os.path.join(self.json_dir, 'person2.json')

        try:
            with open(json1) as f:
                person1 = json.load(f).get('guest1', {})
        except Exception as e:
            rospy.logwarn('[IntroduceGuests] Could not load person1.json: %s', e)
            person1 = None

        try:
            with open(json2) as f:
                person2 = json.load(f).get('guest1', {})
        except Exception as e:
            rospy.logwarn('[IntroduceGuests] Could not load person2.json: %s', e)
            person2 = None

        if not person1 or not person2:
            rospy.logerr('[IntroduceGuests] Missing guest information')
            return self._retry()

        name1  = person1.get('name', 'Guest 1')
        drink1 = person1.get('drink', 'unknown drink')
        name2  = person2.get('name', 'Guest 2')
        drink2 = person2.get('drink', 'unknown drink')

        # Fill in default sides for any unrecognised guests so the robot
        # still looks at someone rather than staring straight ahead
        if name1 not in guest_positions:
            guest_positions[name1] = 'left'
        if name2 not in guest_positions:
            guest_positions[name2] = 'right'

        rospy.loginfo('[IntroduceGuests] %s (%s) | %s (%s) | positions: %s',
                      name1, drink1, name2, drink2, guest_positions)

        # --- Introduce guest 2 to guest 1 ---
        self._look_at(name1, guest_positions)
        self._say(f"{name1}, I would like to introduce you to {name2}, "
                  f"whose favorite drink is {drink2}.")

        rospy.sleep(0.8)

        # --- Introduce guest 1 to guest 2 ---
        self._look_at(name2, guest_positions)
        self._say(f"{name2}, I would like to introduce you to {name1}, "
                  f"whose favorite drink is {drink1}.")

        # --- Return head to neutral ---
        _send_head(PAN_NEUTRAL, 0.0)

        rospy.loginfo('[IntroduceGuests] Introductions completed')
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[IntroduceGuests] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'
