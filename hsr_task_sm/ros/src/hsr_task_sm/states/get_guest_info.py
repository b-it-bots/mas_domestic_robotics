#!/usr/bin/env python3
"""
GetGuestInfo state - Gets guest name and drink via speech recognition + LLM.

Uses Whisper STT service and voicebot/LLM prompt service.
Saves guest info to JSON files for later introduction.
"""

import os
import json
import rospy
import smach

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerRequest

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
        self.say = rospy.Publisher('/say', String, queue_size=10)
        
        # Wait for services
        rospy.loginfo('[GetGuestInfo] Waiting for speech_recognize service...')
        try:
            rospy.wait_for_service('speech_recognize', timeout=10.0)
            self.stt_service = rospy.ServiceProxy('speech_recognize', Trigger)
            rospy.loginfo('[GetGuestInfo] STT service connected.')
        except rospy.ROSException:
            rospy.logwarn('[GetGuestInfo] STT service not available.')
            self.stt_service = None
            
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
        self.say.publish(String(data=text))
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
                # contextualized = f'[Talking with guest number {self.guest_number}] {text}'
                result = self.llm_service(prompt=text)
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
                
                # Say reply from LLM
                self._say(reply)
                
                # Set output userdata
                userdata.guest_name = name
                userdata.guest_drink = drink
                
                self.retry_count = 0
                return 'succeeded'
            else:
                rospy.logwarn('[GetGuestInfo] Name or drink missing, asking again...')
                self._say("I am sorry I didnt get it can you repeat your name and favorite drink")
                self.mic_control_pub.publish(Bool(data=True))
                rospy.sleep(1.0)


            
        
        # Re-enable mic before returning
        self.mic_control_pub.publish(Bool(data=True))
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[GetGuestInfo] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'







# #!/usr/bin/env python3
# """
# GetGuestInfo state - Gets guest name and drink via speech recognition + LLM.

# Uses Whisper STT service and voicebot/LLM prompt service.
# Saves guest info to JSON files for later introduction.
# """

# import os
# import json
# import rospy
# import smach

# from std_msgs.msg import String, Bool
# from std_srvs.srv import Trigger, TriggerRequest

# # Try to import voicebot service (may not be available)
# try:
#     from hsr_task_sm.srv import VoicebotPrompt
#     LLM_SERVICE_AVAILABLE = True
# except ImportError:
#     LLM_SERVICE_AVAILABLE = False
#     rospy.logwarn('[GetGuestInfo] hsr_task_sm.srv.VoicebotPrompt not available')


# class GetGuestInfo(smach.State):
#     """
#     Get guest information (name and favorite drink) via speech.
    
#     Uses:
#         - speech_recognize service (Whisper STT)
#         - voicebot/prompt service (LLM for extraction)
    
#     Params:
#         guest_number: 1 or 2 (which guest we're getting info for)
#         json_dir: directory to save guest JSON files
#         retries: max retries before failing
    
#     Outcomes:
#         succeeded               - guest info saved successfully
#         failed                  - temporary failure (retry)
#         failed_after_retrying   - max retries exhausted
    
#     Output keys:
#         guest_name  - extracted guest name
#         guest_drink - extracted favorite drink
#     """

#     def __init__(self,
#                  guest_number=1,
#                  json_dir='/tmp/hri_guests',
#                  timeout=30.0,
#                  retries=3):
#         smach.State.__init__(
#             self,
#             outcomes=['succeeded', 'failed', 'failed_after_retrying'],
#             output_keys=['guest_name', 'guest_drink']
#         )
#         self.guest_number = guest_number
#         self.json_dir = json_dir
#         self.timeout = timeout
#         self.retries = retries
#         self.retry_count = 0
        
#         # Create output directory
#         os.makedirs(self.json_dir, exist_ok=True)
        
#         # Publishers
#         self.mic_control_pub = rospy.Publisher('/condition_record', Bool, queue_size=10)
#         self.say = rospy.Publisher('/say', String, queue_size=10)
        
#         # Wait for services
#         rospy.loginfo('[GetGuestInfo] Waiting for speech_recognize service...')
#         try:
#             rospy.wait_for_service('speech_recognize', timeout=10.0)
#             self.stt_service = rospy.ServiceProxy('speech_recognize', Trigger)
#             rospy.loginfo('[GetGuestInfo] STT service connected.')
#         except rospy.ROSException:
#             rospy.logwarn('[GetGuestInfo] STT service not available.')
#             self.stt_service = None
            
#         rospy.loginfo('[GetGuestInfo] Waiting for voicebot/prompt service...')
#         if LLM_SERVICE_AVAILABLE:
#             try:
#                 rospy.wait_for_service('voicebot/prompt', timeout=10.0)
#                 self.llm_service = rospy.ServiceProxy('voicebot/prompt', VoicebotPrompt)
#                 rospy.loginfo('[GetGuestInfo] LLM service connected.')
#             except rospy.ROSException:
#                 rospy.logwarn('[GetGuestInfo] LLM service not available.')
#                 self.llm_service = None
#         else:
#             rospy.logwarn('[GetGuestInfo] LLM service module not imported.')
#             self.llm_service = None
    
#     def _say(self, text):
#         """Publish text to /say topic."""
#         self.say.publish(String(data=text))
#         # Calculate delay based on word count
#         num_words = len(text.split())
#         delay = min(0.5, num_words * 0.4)
#         rospy.sleep(delay)


#     def _confirm_info(self, name, drink):
#         """
#         Ask user to confirm extracted name and drink.
#         Returns True if confirmed, False otherwise.
#         """
#         confirm_text = f"So your name is {name} and your favorite drink is {drink}. Is that correct?"
#         self._say(confirm_text)

#         # Enable mic to listen for confirmation
#         self.mic_control_pub.publish(Bool(data=True))

#         try:
#             response = self.stt_service(TriggerRequest())

#             if not response.success:
#                 return False

#             answer = response.message.lower()
#             rospy.loginfo("[GetGuestInfo] Confirmation response: %s", answer)

#             positive = ["yes", "correct", "right", "yeah", "yep"]
#             negative = ["no", "wrong", "incorrect", "nope"]

#             if any(p in answer for p in positive):
#                 return True
#             if any(n in answer for n in negative):
#                 return False

#         except rospy.ServiceException:
#             pass

#         return False

#     def execute(self, userdata):
#         if self.stt_service is None or self.llm_service is None:
#             rospy.logerr('[GetGuestInfo] Required services not available')
#             return self._retry()
        
#         rospy.loginfo('[GetGuestInfo] Getting info for guest %d', self.guest_number)
        
#         # Enable mic
#         self.mic_control_pub.publish(Bool(data=True))
        
#         max_attempts = 5  # attempts within this execute call
#         for attempt in range(max_attempts):
#             # Call Whisper STT
#             try:
#                 rospy.loginfo('[GetGuestInfo] Listening for speech...')
#                 response = self.stt_service(TriggerRequest())
                
#                 if not response.success or not response.message.strip():
#                     rospy.logwarn('[GetGuestInfo] No transcription received')
#                     rospy.sleep(1.0)
#                     continue
                
#                 text = response.message.strip()
#                 rospy.loginfo('[GetGuestInfo] Heard: "%s"', text)
                
#             except rospy.ServiceException as e:
#                 rospy.logerr('[GetGuestInfo] STT service failed: %s', e)
#                 continue
            
#             # Disable mic while processing
#             self.mic_control_pub.publish(Bool(data=False))
            
#             # Send to LLM for extraction (include guest number so voicebot fills correct slot)
#             try:
#                 rospy.loginfo('[GetGuestInfo] Sending to LLM...')
#                 contextualized = f'[Talking with guest number {self.guest_number}] {text}'
#                 result = self.llm_service(prompt=contextualized)
#                 reply = result.response
#                 self._say(reply)
#                 guests = json.loads(result.guests_json)
                
#             except (rospy.ServiceException, json.JSONDecodeError) as e:
#                 rospy.logwarn('[GetGuestInfo] LLM/parsing failed: %s', e)
#                 self.mic_control_pub.publish(Bool(data=True))
#                 continue
            
#             # Check if we got valid info
#             guest_key = f'guest{self.guest_number}' if f'guest{self.guest_number}' in guests else 'guest1'
#             guest = guests.get(guest_key, {})
#             name = guest.get('name')
#             drink = guest.get('drink')


#             if name and drink:
#                 rospy.loginfo('[GetGuestInfo] Extracted: %s wants %s', name, drink)

#                 confirmed = self._confirm_info(name, drink)

#                 if confirmed:
#                     rospy.loginfo('[GetGuestInfo] Information confirmed')

#                     # Save to JSON
#                     json_path = os.path.join(self.json_dir, f'person{self.guest_number}.json')
#                     with open(json_path, 'w') as f:
#                         json.dump({'guest1': guest}, f, indent=4)

#                     rospy.loginfo('[GetGuestInfo] Saved to %s', json_path)

#                     self._say(f"Nice to meet you {name}. I will remember that you like {drink}.")

#                     userdata.guest_name = name
#                     userdata.guest_drink = drink

#                     self.retry_count = 0
#                     return 'succeeded'

#                 else:
#                     rospy.logwarn('[GetGuestInfo] User said info was incorrect')
#                     self._say("Sorry about that. Could you please tell me your name and favorite drink again?")
#                     self.mic_control_pub.publish(Bool(data=True))
#                     rospy.sleep(1.0)
#                     continue
            
#             # if name and drink:
#             #     rospy.loginfo('[GetGuestInfo] Guest %d: %s wants %s', 
#             #                   self.guest_number, name, drink)
                
#             #     # Save to JSON
#             #     json_path = os.path.join(self.json_dir, f'person{self.guest_number}.json')
#             #     with open(json_path, 'w') as f:
#             #         json.dump({'guest1': guest}, f, indent=4)
#             #     rospy.loginfo('[GetGuestInfo] Saved to %s', json_path)
                
#             #     # Say reply from LLM
#             #     self._say(reply)
                
#             #     # Set output userdata
#             #     userdata.guest_name = name
#             #     userdata.guest_drink = drink
                
#             #     self.retry_count = 0
#             #     return 'succeeded'
#             # else:
#             #     rospy.logwarn('[GetGuestInfo] Name or drink missing, asking again...')
#             #     self._say("Sorry, I didn't catch that. Could you please tell me your name and favorite drink again?")
#             #     self.mic_control_pub.publish(Bool(data=True))
#             #     rospy.sleep(1.0)
        
#         # Re-enable mic before returning
#         self.mic_control_pub.publish(Bool(data=True))
#         return self._retry()

#     def _retry(self):
#         if self.retry_count >= self.retries:
#             self.retry_count = 0
#             return 'failed_after_retrying'
#         self.retry_count += 1
#         rospy.logwarn('[GetGuestInfo] Retry %d/%d', self.retry_count, self.retries)
#         return 'failed'