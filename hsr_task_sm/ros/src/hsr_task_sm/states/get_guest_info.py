#!/usr/bin/env python3
"""
GetGuestInfo state - Gets guest name and drink via speech recognition + LLM.

Uses Whisper STT service and voicebot/LLM prompt service.
Saves guest info to JSON files for later introduction.
"""

import os
import base64
import json
import rospy
import smach

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# Try to import voicebot service (may not be available)
try:
    from hsr_task_sm.srv import VoicebotPrompt
    LLM_SERVICE_AVAILABLE = True
except ImportError:
    LLM_SERVICE_AVAILABLE = False
    rospy.logwarn('[GetGuestInfo] hsr_task_sm.srv.VoicebotPrompt not available')


class GetGuestInfo(smach.State):
    """
    Get guest information (name and favorite drink) via speech.
    
    Uses:
        - speech_recognize service (Whisper STT)
        - voicebot/prompt service (LLM for extraction)
    
    Params:
        guest_number: 1 or 2 (which guest we're getting info for)
        json_dir: directory to save guest JSON files
        retries: max retries before failing
    
    Outcomes:
        succeeded               - guest info saved successfully
        failed                  - temporary failure (retry)
        failed_after_retrying   - max retries exhausted
    
    Output keys:
        guest_name  - extracted guest name
        guest_drink - extracted favorite drink
    """

    def __init__(self,
                 guest_number=1,
                 json_dir='/tmp/hri_guests',
                 timeout=30.0,
                 retries=3):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['guest_name', 'guest_drink']
        )
        self.guest_number = guest_number
        self.json_dir = json_dir
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0
        
        # Create output directory
        os.makedirs(self.json_dir, exist_ok=True)
        
        # Publishers
        self.mic_control_pub = rospy.Publisher('/condition_record', Bool, queue_size=10)
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)
        
        # Wait for services
        rospy.loginfo('[GetGuestInfo] Waiting for speech_recognize service...')
        try:
            rospy.wait_for_service('speech_recognize', timeout=10.0)
            self.stt_service = rospy.ServiceProxy('speech_recognize', Trigger)
            rospy.loginfo('[GetGuestInfo] STT service connected.')
        except rospy.ROSException:
            rospy.logwarn('[GetGuestInfo] STT service not available.')
            self.stt_service = None
            
        # ReID service for saving guest appearance (preferred) or VLM fallback
        self.vlm_service = None
        self._bridge = CvBridge() if CV_AVAILABLE else None
        try:
            from hsr_task_sm.srv import VLMQuery
            # Try FastReID server first, fall back to VLM server
            for svc_name in ['/reid/query', '/vlm/query']:
                try:
                    rospy.wait_for_service(svc_name, timeout=3.0)
                    self.vlm_service = rospy.ServiceProxy(svc_name, VLMQuery)
                    rospy.loginfo('[GetGuestInfo] Using %s for appearance saving.', svc_name)
                    break
                except Exception:
                    continue
            if self.vlm_service and self.guest_number == 1:
                self.vlm_service(query_type='clear_faces', image_base64='', context='')
                rospy.loginfo('[GetGuestInfo] Cleared previous guest features for new run.')
        except Exception:
            rospy.logwarn('[GetGuestInfo] No recognition service available — skipping image save.')

        rospy.loginfo('[GetGuestInfo] Waiting for voicebot/prompt service...')
        if LLM_SERVICE_AVAILABLE:
            try:
                rospy.wait_for_service('voicebot/prompt', timeout=10.0)
                self.llm_service = rospy.ServiceProxy('voicebot/prompt', VoicebotPrompt)
                rospy.loginfo('[GetGuestInfo] LLM service connected.')
            except rospy.ROSException:
                rospy.logwarn('[GetGuestInfo] LLM service not available.')
                self.llm_service = None
        else:
            rospy.logwarn('[GetGuestInfo] LLM service module not imported.')
            self.llm_service = None

    def _say(self, text):
        """Publish text to /say topic."""
        self.say_pub.publish(String(data=text))
        # Calculate delay based on word count
        num_words = len(text.split())
        delay = max(0.5, num_words * 0.4)
        rospy.sleep(delay)

    def execute(self, userdata):
        if self.stt_service is None or self.llm_service is None:
            rospy.logerr('[GetGuestInfo] Required services not available')
            return self._retry()
        
        rospy.loginfo('[GetGuestInfo] Getting info for guest %d', self.guest_number)
        
        # Enable mic
        self.mic_control_pub.publish(Bool(data=True))
        
        max_attempts = 5  # attempts within this execute call
        for attempt in range(max_attempts):
            # Call Whisper STT
            try:
                rospy.loginfo('[GetGuestInfo] Listening for speech...')
                response = self.stt_service(TriggerRequest())
                
                if not response.success or not response.message.strip():
                    rospy.logwarn('[GetGuestInfo] No transcription received')
                    rospy.sleep(1.0)
                    continue
                
                text = response.message.strip()
                rospy.loginfo('[GetGuestInfo] Heard: "%s"', text)
                
            except rospy.ServiceException as e:
                rospy.logerr('[GetGuestInfo] STT service failed: %s', e)
                continue
            
            # Disable mic while processing
            self.mic_control_pub.publish(Bool(data=False))
            
            # Send to LLM for extraction (include guest number so voicebot fills correct slot)
            try:
                rospy.loginfo('[GetGuestInfo] Sending to LLM...')
                contextualized = f'[Talking with guest number {self.guest_number}] {text}'
                result = self.llm_service(prompt=contextualized)
                reply = result.response
                guests = json.loads(result.guests_json)
                
            except (rospy.ServiceException, json.JSONDecodeError) as e:
                rospy.logwarn('[GetGuestInfo] LLM/parsing failed: %s', e)
                self.mic_control_pub.publish(Bool(data=True))
                continue
            
            # Check if we got valid info
            guest_key = f'guest{self.guest_number}' if f'guest{self.guest_number}' in guests else 'guest1'
            guest = guests.get(guest_key, {})
            name = guest.get('name')
            drink = guest.get('drink')
            
            if name and drink:
                rospy.loginfo('[GetGuestInfo] Guest %d: %s wants %s', 
                              self.guest_number, name, drink)
                
                # Save to JSON
                json_path = os.path.join(self.json_dir, f'person{self.guest_number}.json')
                with open(json_path, 'w') as f:
                    json.dump({'guest1': guest}, f, indent=4)
                rospy.loginfo('[GetGuestInfo] Saved to %s', json_path)

                # Capture and save guest appearance image for later recognition
                self._save_guest_image(name)

                # Say reply from LLM
                self._say(reply)

                # Set output userdata
                userdata.guest_name = name
                userdata.guest_drink = drink

                self.retry_count = 0
                return 'succeeded'
            else:
                rospy.logwarn('[GetGuestInfo] Name or drink missing, asking again...')
                self._say(reply)
                self.mic_control_pub.publish(Bool(data=True))
                rospy.sleep(1.0)
        
        # Re-enable mic before returning
        self.mic_control_pub.publish(Bool(data=True))
        return self._retry()

    def _set_head(self, pan, tilt, duration=1.2):
        """Send a head trajectory command and wait."""
        pub = rospy.Publisher('/hsrb/head_trajectory_controller/command',
                              JointTrajectory, queue_size=1, latch=True)
        rospy.sleep(0.1)
        traj = JointTrajectory()
        traj.joint_names = ['head_pan_joint', 'head_tilt_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        pub.publish(traj)
        rospy.sleep(duration + 0.3)

    def _save_guest_image(self, name):
        """Tilt head up to face level, capture image, save to ReID server, restore head."""
        if not self.vlm_service or not CV_AVAILABLE:
            return
        try:
            # Tilt head up slightly so the person's face is centred in the frame
            self._set_head(pan=0.0, tilt=-0.20)   # -0.20 rad = looking slightly upward
            rospy.sleep(0.3)   # let image stabilise

            msg = rospy.wait_for_message(
                '/hsrb/head_rgbd_sensor/rgb/image_rect_color', RosImage, timeout=3.0)
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
            resp = self.vlm_service(query_type='save_face', image_base64=img_b64, context=name)
            if resp.success:
                rospy.loginfo('[GetGuestInfo] Saved appearance image for "%s"', name)
            else:
                rospy.logwarn('[GetGuestInfo] VLM save_face failed: %s', resp.reason)

            # Restore head to neutral
            self._set_head(pan=0.0, tilt=0.0)
        except Exception as e:
            rospy.logwarn('[GetGuestInfo] Could not save guest image: %s', e)
            try:
                self._set_head(pan=0.0, tilt=0.0)
            except Exception:
                pass

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[GetGuestInfo] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'


class RecognizePerson(smach.State):
    """
    Capture a camera image and ask the VLM server to identify the person
    by matching against saved face/appearance images.

    Params:
        camera_topic (str) - RGB image topic (default: head RGBD sensor)
        retries      (int) - max retries on failure (default: 2)

    Output keys:
        recognized_name - name of the identified person, or 'unknown'

    Outcomes: succeeded | failed | failed_after_retrying
    """

    def __init__(self,
                 camera_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['recognized_name']
        )
        self.camera_topic = camera_topic
        self.retries = retries
        self.retry_count = 0
        self._bridge = CvBridge() if CV_AVAILABLE else None
        self.vlm_service = None

        try:
            from hsr_task_sm.srv import VLMQuery
            # Prefer FastReID server, fall back to VLM server
            for svc_name in ['/reid/query', '/vlm/query']:
                try:
                    rospy.wait_for_service(svc_name, timeout=3.0)
                    self.vlm_service = rospy.ServiceProxy(svc_name, VLMQuery)
                    rospy.loginfo('[RecognizePerson] Using %s.', svc_name)
                    break
                except Exception:
                    continue
        except Exception as e:
            rospy.logwarn('[RecognizePerson] No recognition service available: %s', e)

    def execute(self, userdata):
        userdata.recognized_name = 'unknown'

        if not self.vlm_service or not CV_AVAILABLE:
            rospy.logerr('[RecognizePerson] VLM service or CV not available')
            return self._retry()

        try:
            msg = rospy.wait_for_message(self.camera_topic, RosImage, timeout=5.0)
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            img_b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
        except Exception as e:
            rospy.logerr('[RecognizePerson] Camera capture failed: %s', e)
            return self._retry()

        try:
            resp = self.vlm_service(
                query_type='recognize_person',
                image_base64=img_b64,
                context=''
            )
            if resp.success:
                rospy.loginfo('[RecognizePerson] Recognized: %s', resp.answer)
                userdata.recognized_name = resp.answer
                self.retry_count = 0
                return 'succeeded'
            else:
                rospy.logwarn('[RecognizePerson] VLM failed: %s', resp.reason)
                userdata.recognized_name = 'unknown'
                return self._retry()
        except Exception as e:
            rospy.logerr('[RecognizePerson] Service call failed: %s', e)
            return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'
