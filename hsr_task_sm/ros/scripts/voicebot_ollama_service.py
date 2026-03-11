#!/usr/bin/env python3
"""
Voicebot LLM Service using Ollama

This service runs on the slave laptop and provides conversational AI
capabilities using Ollama (local LLM). It can extract structured information
(guest names, drinks) from natural conversation.

Slave Laptop Setup:
    1. Install Ollama:
       curl -fsSL https://ollama.com/install.sh | sh
    
    2. Pull a model:
       ollama pull llama3.2
    
    3. Install Python dependencies:
       pip install ollama
    
    4. Set ROS_MASTER_URI to point to robot:
       export ROS_MASTER_URI=http://<robot_ip>:11311
       export ROS_IP=<slave_laptop_ip>
    
    5. Run this service:
       rosrun hsr_task_sm voicebot_ollama_service.py

ROS Interface:
    Service: /voicebot/prompt (hsr_task_sm/VoicebotPrompt)
    Publisher: /say (std_msgs/String) - for TTS output
"""

import json
import re
import rospy
from std_msgs.msg import String

# Try to import ollama
try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False
    rospy.logwarn("Ollama not installed. Install with: pip install ollama")


# =============================================================================
# GERMAN OPEN 2026 - OFFICIAL COMPETITION DATA
# Reference: https://github.com/RoboCupAtHome/GermanOpen2026
# =============================================================================
VALID_NAMES = [
    "Oliver", "Charlotte", "Henry", "Amelia", "Jack", "Sophia", "Thomas", "Emily",
    "James", "Isabella", "William", "Ava", "George", "Mia", "Harry", "Lily",
    "Samuel", "Grace", "Benjamin", "Chloe", "Daniel", "Ella", "Matthew", "Scarlett",
    "Joseph", "Harper", "David", "Evelyn", "Lucas", "Abigail", "Alexander", "Madison",
    "Michael", "Avery", "Ethan", "Sofia", "Jacob", "Aria", "Logan", "Zoey",
    "Noah", "Hannah", "Ryan", "Stella", "Nathan", "Victoria", "Caleb"
]

VALID_DRINKS = [
    "water", "red bull", "milk", "orange juice", "coke", "ice tea", "coffee creamer"
]

# Alternative spellings/phrases people might use
DRINK_ALIASES = {
    "redbull": "red bull",
    "red-bull": "red bull",
    "oj": "orange juice",
    "orange": "orange juice",
    "coca cola": "coke",
    "coca-cola": "coke",
    "cola": "coke",
    "pepsi": "coke",
    "iced tea": "ice tea",
    "icetea": "ice tea",
    "tea": "ice tea",
    "creamer": "coffee creamer",
    "coffee": "coffee creamer",
}


# Default system prompt for receptionist task
DEFAULT_SYSTEM_PROMPT = """You are a friendly robot receptionist at a party. Your job is to:
1. Greet guests warmly
2. Ask for their name if not provided
3. Ask for their favorite drink if not provided
4. Remember information about guests

Keep responses SHORT (1-2 sentences). Be conversational and friendly.
When you have both name and drink, confirm and say "Please follow me to meet the host."

Important: If the guest hasn't given their name, ask for it.
If they haven't said their favorite drink, ask about it.

Valid drinks at this party: water, red bull, milk, orange juice, coke, ice tea, coffee creamer.
"""


class VoicebotService:
    """
    Voicebot LLM Service using Ollama
    
    Parameters:
        ~model (str): Ollama model name (default: llama3.2)
        ~system_prompt (str): System prompt for the LLM
        ~max_history (int): Maximum conversation history to maintain
    """
    
    def __init__(self):
        rospy.init_node('voicebot_ollama_service', anonymous=False)
        
        # Parameters
        self.model_name = rospy.get_param('~model', 'llama3.2')
        self.system_prompt = rospy.get_param('~system_prompt', DEFAULT_SYSTEM_PROMPT)
        self.max_history = rospy.get_param('~max_history', 10)
        
        # Guest memory
        self.guests = {
            'guest1': {'name': None, 'drink': None},
            'guest2': {'name': None, 'drink': None}
        }
        self.current_guest = 'guest1'
        
        # Conversation history
        self.conversation_history = [
            {'role': 'system', 'content': self.system_prompt}
        ]
        
        # Publishers
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)
        
        # Service - we need to import the service type
        # For now, use a simple approach with message callback
        self.prompt_sub = rospy.Subscriber('/voicebot/prompt_request', String, self._handle_prompt)
        self.response_pub = rospy.Publisher('/voicebot/response', String, queue_size=10)
        
        # Also create the actual service
        try:
            from hsr_task_sm.srv import VoicebotPrompt, VoicebotPromptResponse
            self.service = rospy.Service('voicebot/prompt', VoicebotPrompt, self._handle_service)
            rospy.loginfo("VoicebotPrompt service registered")
        except ImportError:
            rospy.logwarn("VoicebotPrompt service type not found, using topic-based interface")
            self.service = None
        
        rospy.loginfo("Voicebot Ollama Service ready")
        rospy.loginfo(f"  Model: {self.model_name}")
        rospy.loginfo(f"  Ollama available: {OLLAMA_AVAILABLE}")
    
    def _extract_guest_info(self, text):
        """
        Extract guest name and drink from text using LLM + regex fallback.
        
        Returns:
            dict: {'guest1': {'name': str|None, 'drink': str|None}, ...}
        """
        # Try LLM extraction first
        extraction_prompt = f"""
Return ONLY valid JSON (no markdown, no comments) in exactly this schema:
{{
  "guest1": {{"name": null, "drink": null}},
  "guest2": {{"name": null, "drink": null}}
}}

Extract up to two guests' names and favorite drinks from the text.
If missing, keep null.

Text: {text}
"""
        
        data = None
        
        if OLLAMA_AVAILABLE:
            try:
                resp = ollama.chat(
                    model=self.model_name,
                    messages=[
                        {'role': 'system', 'content': 'You extract structured data. Return only JSON.'},
                        {'role': 'user', 'content': extraction_prompt},
                    ],
                    stream=False
                )
                
                content = self._get_response_content(resp)
                content = self._strip_code_fences(content)
                data = json.loads(content)
                
            except Exception as e:
                rospy.logwarn(f"LLM extraction failed: {e}")
        
        # Regex fallback
        if not data:
            data = {'guest1': {'name': None, 'drink': None}, 'guest2': {'name': None, 'drink': None}}
        
        # Extract name with regex
        name = None
        patterns = [
            r"\bmy name is\s+([A-Za-z][A-Za-z'-]{1,30})\b",
            r"\b(?:i'm|i am)\s+([A-Za-z][A-Za-z'-]{1,30})\b",
            r"\bcall me\s+([A-Za-z][A-Za-z'-]{1,30})\b",
        ]
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                name = match.group(1)
                break
        
        # Extract drink with regex
        drink = None
        drink_patterns = [
            r"\bmy favorite drink is\s+([A-Za-z][A-Za-z0-9' -]{1,40})\b",
            r"\bi (?:like|love|prefer|want)\s+([A-Za-z][A-Za-z0-9' -]{1,40})\b",
            r"\b(?:favorite|fav) drink (?:is )?\s*([A-Za-z][A-Za-z0-9' -]{1,40})\b",
        ]
        for pattern in drink_patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                drink = match.group(1).strip()
                break
        
        # Merge regex results if LLM missed them
        if name and not data.get('guest1', {}).get('name'):
            data['guest1']['name'] = name
        if drink and not data.get('guest1', {}).get('drink'):
            data['guest1']['drink'] = drink
        
        return data
    
    def _get_response_content(self, resp):
        """Extract content from Ollama response (handles different formats)."""
        if isinstance(resp, dict):
            msg = resp.get('message', {})
            if isinstance(msg, dict):
                return msg.get('content', '')
            return ''
        if hasattr(resp, 'message') and resp.message:
            return getattr(resp.message, 'content', '')
        return ''
    
    def _strip_code_fences(self, s):
        """Remove markdown code fences from string."""
        s = s.strip()
        if s.startswith('```'):
            s = re.sub(r'^```[a-zA-Z]*\n?', '', s)
            s = s.rstrip('`').strip()
        return s
    
    def _get_llm_response(self, user_input):
        """
        Get conversational response from Ollama LLM.
        
        Returns:
            str: LLM response text
        """
        if not OLLAMA_AVAILABLE:
            return "I'm having trouble with my language processing. Could you repeat that?"
        
        # Update conversation history
        self.conversation_history.append({'role': 'user', 'content': user_input})
        
        # Add memory context
        memory_context = f"""
Current guest information:
Guest 1: Name={self.guests['guest1']['name']}, Drink={self.guests['guest1']['drink']}
Guest 2: Name={self.guests['guest2']['name']}, Drink={self.guests['guest2']['drink']}
"""
        
        # Build messages with memory
        messages = [
            {'role': 'system', 'content': self.system_prompt},
            {'role': 'system', 'content': memory_context}
        ] + self.conversation_history[1:]  # Skip original system prompt
        
        try:
            response_text = ""
            
            # Stream response
            stream = ollama.chat(
                model=self.model_name,
                messages=messages,
                stream=True
            )
            
            for chunk in stream:
                content = self._get_response_content(chunk)
                if content:
                    response_text += content
            
            # Add to history
            self.conversation_history.append({'role': 'assistant', 'content': response_text})
            
            # Trim history if too long
            if len(self.conversation_history) > self.max_history + 1:
                self.conversation_history = [self.conversation_history[0]] + self.conversation_history[-self.max_history:]
            
            return response_text
            
        except Exception as e:
            rospy.logerr(f"LLM error: {e}")
            return "I'm sorry, I encountered an error. Could you repeat that?"
    
    def _handle_prompt(self, msg):
        """Handle prompt via topic (fallback interface)."""
        text = msg.data
        
        # Extract guest info
        info = self._extract_guest_info(text)
        self._update_guest_memory(info)
        
        # Get LLM response
        response = self._get_llm_response(text)
        
        # Publish response
        result = {
            'response': response,
            'guests': self.guests
        }
        self.response_pub.publish(json.dumps(result))
        
        # Also publish to /say for TTS
        self.say_pub.publish(response)
    
    def _handle_service(self, req):
        """Handle VoicebotPrompt service request."""
        from hsr_task_sm.srv import VoicebotPromptResponse
        
        text = req.prompt
        
        # Extract guest info
        info = self._extract_guest_info(text)
        self._update_guest_memory(info)
        
        # Get LLM response
        response = self._get_llm_response(text)
        
        return VoicebotPromptResponse(
            response=response,
            guests_json=json.dumps(self.guests)
        )
    
    def _update_guest_memory(self, info):
        """Update guest memory with extracted info."""
        if info.get('guest1'):
            if info['guest1'].get('name'):
                self.guests['guest1']['name'] = info['guest1']['name']
            if info['guest1'].get('drink'):
                self.guests['guest1']['drink'] = info['guest1']['drink']
        
        if info.get('guest2'):
            if info['guest2'].get('name'):
                self.guests['guest2']['name'] = info['guest2']['name']
            if info['guest2'].get('drink'):
                self.guests['guest2']['drink'] = info['guest2']['drink']
        
        rospy.loginfo(f"Guest memory: {self.guests}")
    
    def reset_conversation(self):
        """Reset conversation for new guest."""
        self.conversation_history = [
            {'role': 'system', 'content': self.system_prompt}
        ]
        
        # Cycle to next guest
        if self.current_guest == 'guest1':
            self.current_guest = 'guest2'
        else:
            self.current_guest = 'guest1'
    
    def run(self):
        """Run the service."""
        rospy.spin()


def main():
    try:
        service = VoicebotService()
        service.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
