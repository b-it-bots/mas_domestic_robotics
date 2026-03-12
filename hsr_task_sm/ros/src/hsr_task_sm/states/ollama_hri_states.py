#!/usr/bin/env python3
"""
Ollama-based HRI States

SMACH states that integrate with the Whisper STT and Voicebot Ollama services
running on a slave laptop connected via ROS network.

Architecture:
    [Robot] <--ROS Topics/Services--> [Slave Laptop]
                                       - whisper_stt_service.py (STT)
                                       - voicebot_ollama_service.py (LLM)

These states assume services are available at:
    - /speech_recognize (std_srvs/Trigger) - Whisper STT
    - /voicebot/prompt (hsr_task_sm/VoicebotPrompt) - Ollama LLM
    - /say (std_msgs/String) - TTS output
"""

import json
import rospy
import smach
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerRequest


class ListenWithWhisper(smach.State):
    """
    Listen for speech using Whisper STT service on slave laptop.
    
    Outcomes:
        succeeded: Speech recognized, text in output_keys
        timeout: No speech detected after timeout
        failed: Service call failed
    
    Output Keys:
        recognized_text: The transcribed speech
    """
    
    def __init__(self, timeout=30.0, service_name='/speech_recognize'):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'timeout', 'failed'],
            output_keys=['recognized_text']
        )
        self.timeout = timeout
        self.service_name = service_name
    
    def execute(self, userdata):
        rospy.loginfo('[ListenWithWhisper] Waiting for speech...')
        
        try:
            # Wait for service
            rospy.wait_for_service(self.service_name, timeout=10.0)
            speech_service = rospy.ServiceProxy(self.service_name, Trigger)
            
            # Call service
            response = speech_service(TriggerRequest())
            
            if response.success and response.message.strip():
                userdata.recognized_text = response.message.strip()
                rospy.loginfo(f'[ListenWithWhisper] Heard: {userdata.recognized_text}')
                return 'succeeded'
            else:
                rospy.logwarn('[ListenWithWhisper] No speech recognized')
                return 'timeout'
                
        except rospy.ServiceException as e:
            rospy.logerr(f'[ListenWithWhisper] Service call failed: {e}')
            return 'failed'
        except rospy.ROSException as e:
            rospy.logerr(f'[ListenWithWhisper] Service not available: {e}')
            return 'failed'


class GetVoicebotResponse(smach.State):
    """
    Send text to Voicebot LLM service and get response.
    Also extracts guest information (name, drink).
    
    Outcomes:
        succeeded: Got response from LLM
        guest_complete: Guest info (name + drink) is complete
        failed: Service call failed
    
    Input Keys:
        user_text: Text to send to voicebot
    
    Output Keys:
        bot_response: LLM response text
        guest_name: Extracted guest name (if any)
        guest_drink: Extracted guest drink (if any)
        guests_json: Full guest info JSON
    """
    
    def __init__(self, service_name='/voicebot/prompt'):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'guest_complete', 'failed'],
            input_keys=['user_text'],
            output_keys=['bot_response', 'guest_name', 'guest_drink', 'guests_json']
        )
        self.service_name = service_name
    
    def execute(self, userdata):
        user_text = userdata.user_text
        rospy.loginfo(f'[GetVoicebotResponse] Sending to LLM: {user_text}')
        
        try:
            # Try service-based interface first
            try:
                from hsr_task_sm.srv import VoicebotPrompt
                rospy.wait_for_service(self.service_name, timeout=5.0)
                proxy = rospy.ServiceProxy(self.service_name, VoicebotPrompt)
                result = proxy(user_text)
                
                userdata.bot_response = result.response
                guests = json.loads(result.guests_json)
                
            except (ImportError, rospy.ROSException):
                # Fall back to topic-based interface
                rospy.loginfo('[GetVoicebotResponse] Using topic-based interface')
                
                # Publish prompt
                prompt_pub = rospy.Publisher('/voicebot/prompt_request', String, queue_size=1)
                rospy.sleep(0.2)
                prompt_pub.publish(user_text)
                
                # Wait for response
                try:
                    response_msg = rospy.wait_for_message('/voicebot/response', String, timeout=30.0)
                    result = json.loads(response_msg.data)
                    userdata.bot_response = result.get('response', '')
                    guests = result.get('guests', {})
                except rospy.ROSException:
                    rospy.logerr('[GetVoicebotResponse] No response received')
                    return 'failed'
            
            # Extract guest info
            guest1 = guests.get('guest1', {})
            userdata.guest_name = guest1.get('name')
            userdata.guest_drink = guest1.get('drink')
            userdata.guests_json = json.dumps(guests)
            
            rospy.loginfo(f'[GetVoicebotResponse] Response: {userdata.bot_response}')
            rospy.loginfo(f'[GetVoicebotResponse] Guest: {userdata.guest_name}, Drink: {userdata.guest_drink}')
            
            # Check if guest info is complete
            if userdata.guest_name and userdata.guest_drink:
                return 'guest_complete'
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f'[GetVoicebotResponse] Error: {e}')
            return 'failed'


class SpeakResponse(smach.State):
    """
    Speak the bot response using TTS.
    
    Outcomes:
        succeeded: Speech published
        failed: Error occurred
    
    Input Keys:
        response_text: Text to speak (or use text parameter)
    """
    
    def __init__(self, text=None, text_key=None, topic='/say'):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['response_text'] if text_key else []
        )
        self.text = text
        self.text_key = text_key
        self.topic = topic
        self.pub = None
    
    def execute(self, userdata):
        # Determine text to speak
        if self.text:
            speak_text = self.text
        elif self.text_key and hasattr(userdata, self.text_key):
            speak_text = getattr(userdata, self.text_key)
        elif hasattr(userdata, 'response_text'):
            speak_text = userdata.response_text
        else:
            speak_text = "I'm not sure what to say."
        
        rospy.loginfo(f'[SpeakResponse] Saying: {speak_text}')
        
        try:
            if not self.pub:
                self.pub = rospy.Publisher(self.topic, String, queue_size=10)
                rospy.sleep(0.2)
            
            self.pub.publish(speak_text)
            
            # Wait based on text length
            words = len(speak_text.split())
            delay = max(1.0, words * 0.3)
            rospy.sleep(delay)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f'[SpeakResponse] Error: {e}')
            return 'failed'


class ControlMicrophone(smach.State):
    """
    Enable or disable microphone on slave laptop.
    
    Outcomes:
        succeeded: Command sent
    """
    
    def __init__(self, enable=True, topic='/condition_record'):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.enable = enable
        self.topic = topic
        self.pub = None
    
    def execute(self, userdata):
        state = "enabled" if self.enable else "disabled"
        rospy.loginfo(f'[ControlMicrophone] Microphone {state}')
        
        if not self.pub:
            self.pub = rospy.Publisher(self.topic, Bool, queue_size=10)
            rospy.sleep(0.2)
        
        self.pub.publish(Bool(data=self.enable))
        return 'succeeded'


class ConversationLoop(smach.State):
    """
    Main conversation loop state - combines listen, process, speak.
    Continues until guest info is complete or max iterations reached.
    
    Outcomes:
        guest_complete: Got name and drink
        max_iterations: Reached max conversation turns
        failed: Error occurred
    
    Output Keys:
        guest_name: Extracted name
        guest_drink: Extracted drink
    """
    
    def __init__(self, max_iterations=10):
        smach.State.__init__(
            self,
            outcomes=['guest_complete', 'max_iterations', 'failed'],
            output_keys=['guest_name', 'guest_drink']
        )
        self.max_iterations = max_iterations
        self.say_pub = None
    
    def execute(self, userdata):
        if not self.say_pub:
            self.say_pub = rospy.Publisher('/say', String, queue_size=10)
            rospy.sleep(0.2)
        
        iteration = 0
        
        while not rospy.is_shutdown() and iteration < self.max_iterations:
            iteration += 1
            rospy.loginfo(f'[ConversationLoop] Turn {iteration}/{self.max_iterations}')
            
            # Listen
            try:
                rospy.wait_for_service('/speech_recognize', timeout=5.0)
                speech_service = rospy.ServiceProxy('/speech_recognize', Trigger)
                response = speech_service(TriggerRequest())
                
                if not response.success or not response.message.strip():
                    rospy.logwarn('[ConversationLoop] No speech, retrying...')
                    self.say_pub.publish("I didn't catch that. Could you repeat?")
                    rospy.sleep(2.0)
                    continue
                
                user_text = response.message.strip()
                rospy.loginfo(f'[ConversationLoop] Heard: {user_text}')
                
            except Exception as e:
                rospy.logerr(f'[ConversationLoop] STT error: {e}')
                return 'failed'
            
            # Get LLM response
            try:
                from hsr_task_sm.srv import VoicebotPrompt
                rospy.wait_for_service('/voicebot/prompt', timeout=5.0)
                proxy = rospy.ServiceProxy('/voicebot/prompt', VoicebotPrompt)
                result = proxy(user_text)
                
                bot_response = result.response
                guests = json.loads(result.guests_json)
                guest1 = guests.get('guest1', {})
                
                name = guest1.get('name')
                drink = guest1.get('drink')
                
            except ImportError:
                # Topic-based fallback
                prompt_pub = rospy.Publisher('/voicebot/prompt_request', String, queue_size=1)
                rospy.sleep(0.2)
                prompt_pub.publish(user_text)
                
                try:
                    response_msg = rospy.wait_for_message('/voicebot/response', String, timeout=30.0)
                    result = json.loads(response_msg.data)
                    bot_response = result.get('response', '')
                    guests = result.get('guests', {})
                    guest1 = guests.get('guest1', {})
                    name = guest1.get('name')
                    drink = guest1.get('drink')
                except Exception as e:
                    rospy.logerr(f'[ConversationLoop] LLM error: {e}')
                    return 'failed'
            except Exception as e:
                rospy.logerr(f'[ConversationLoop] LLM error: {e}')
                return 'failed'
            
            # Speak response
            rospy.loginfo(f'[ConversationLoop] Bot: {bot_response}')
            self.say_pub.publish(bot_response)
            
            # Wait for speech
            words = len(bot_response.split())
            rospy.sleep(max(1.0, words * 0.3))
            
            # Check if complete
            if name and drink:
                userdata.guest_name = name
                userdata.guest_drink = drink
                rospy.loginfo(f'[ConversationLoop] Complete! {name} likes {drink}')
                return 'guest_complete'
        
        return 'max_iterations'


class SaveGuestInfo(smach.State):
    """
    Save guest information to JSON file.
    
    Outcomes:
        succeeded: Info saved
        failed: Error occurred
    
    Input Keys:
        guest_name: Name to save
        guest_drink: Drink to save
        guest_number: Which guest (1 or 2)
    """
    
    def __init__(self, output_dir='/tmp/hri_guests'):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['guest_name', 'guest_drink', 'guest_number']
        )
        self.output_dir = output_dir
    
    def execute(self, userdata):
        import os
        
        name = getattr(userdata, 'guest_name', 'Unknown')
        drink = getattr(userdata, 'guest_drink', 'Unknown')
        number = getattr(userdata, 'guest_number', 1)
        
        rospy.loginfo(f'[SaveGuestInfo] Saving guest {number}: {name}, {drink}')
        
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            
            filepath = os.path.join(self.output_dir, f'person{number}.json')
            data = {
                'guest1': {
                    'name': name,
                    'drink': drink
                }
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            
            rospy.loginfo(f'[SaveGuestInfo] Saved to {filepath}')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f'[SaveGuestInfo] Save failed: {e}')
            return 'failed'
