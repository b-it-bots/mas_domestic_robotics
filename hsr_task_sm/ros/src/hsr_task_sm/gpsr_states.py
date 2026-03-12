#!/usr/bin/env python3
"""
GPSR States - General Purpose Service Robot

Key state: GPSRCommandParser
  - Listens via Whisper STT
  - Sends command to Ollama with a structured parsing prompt
  - Returns the action type as a SMACH outcome so the YAML SM can branch directly

Action outcomes:
    fetch          - pick up object, bring to operator
    find_person    - locate a person in a room
    tell           - go to person, deliver information
    guide          - escort person from A to B
    count_objects  - count items at a location
    count_persons  - count people in a room
    failed         - could not parse / service error
"""

import re
import json
import rospy
import smach
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest


# ---------------------------------------------------------------------------
# Configurable lists (keep in sync with gpsr_challenge.yaml userdata)
# ---------------------------------------------------------------------------
KNOWN_LOCATIONS = [
    'living_room_table', 'living_room_far', 'dining_table',
    'bedroom_entrance', 'door1_inside', 'door1_outside',
    'shelf', 'fridge', 'kitchen', 'bedroom', 'living_room', 'entrance',
]

KNOWN_OBJECTS = [
    'cup', 'bottle', 'plate', 'bowl', 'apple', 'orange',
    'bread', 'book', 'phone', 'snack', 'drink',
]

VALID_ACTIONS = [
    'fetch', 'find_person', 'tell', 'guide',
    'count_objects', 'count_persons',
]


class GPSRCommandParser(smach.State):
    """
    Parse a spoken GPSR command using Whisper STT + Ollama LLM.

    Flow:
        1. Call /speech_recognize (Whisper) to get raw command text
        2. Call /voicebot/prompt (Ollama) with a JSON-extraction prompt
        3. Parse the JSON → action, target, location, target_location, description
        4. Write fields to userdata
        5. Return the action type as a SMACH outcome for direct YAML branching

    Input Keys:  (none required — listens fresh each time)
    Output Keys:
        recognized_text        Raw STT transcript
        parsed_action          One of VALID_ACTIONS
        parsed_target          Object or person name
        parsed_location        Where the robot needs to go first
        parsed_target_location Where to bring / guide to (if applicable)
        command_description    One-sentence human-readable plan

    Outcomes: fetch | find_person | tell | guide |
              count_objects | count_persons | failed
    """

    SYSTEM_PROMPT_TEMPLATE = (
        "[GPSR_PARSE] You are a command interpreter for a domestic service robot. "
        "Parse the command below and respond with ONLY a JSON object — no explanations, "
        "no markdown, just the raw JSON.\n\n"
        "Available action types:\n"
        "  fetch        - pick up an object and bring it to the operator\n"
        "  find_person  - locate a specific person in a room\n"
        "  tell         - navigate to someone and deliver a piece of information\n"
        "  guide        - escort a person from one location to another\n"
        "  count_objects - count objects on a surface or in a room\n"
        "  count_persons - count people in a room\n\n"
        "Known locations: {locations}\n"
        "Known objects: {objects}\n\n"
        "Command: \"{command}\"\n\n"
        "Respond with JSON only:\n"
        '{{"action":"<type>","target":"<object or person or empty>","location":"<where to go first>",'
        '"target_location":"<destination if guiding/delivering or empty>","description":"<one sentence plan>"}}'
    )

    def __init__(self,
                 stt_service='/speech_recognize',
                 llm_service='/voicebot/prompt',
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=VALID_ACTIONS + ['failed'],
            output_keys=[
                'recognized_text',
                'parsed_action',
                'parsed_target',
                'parsed_location',
                'parsed_target_location',
                'command_description',
            ]
        )
        self.stt_service = stt_service
        self.llm_service = llm_service
        self.retries = retries
        self.retry_count = 0
        self._say_pub = None

    # ------------------------------------------------------------------
    def _say(self, text):
        if self._say_pub is None:
            self._say_pub = rospy.Publisher('/say', String, queue_size=5)
            rospy.sleep(0.2)
        rospy.loginfo('[GPSRCommandParser] Say: %s', text)
        self._say_pub.publish(text)
        rospy.sleep(max(1.0, len(text.split()) * 0.3))

    # ------------------------------------------------------------------
    def _listen(self):
        """Call Whisper STT service. Returns text or None."""
        try:
            rospy.wait_for_service(self.stt_service, timeout=10.0)
            svc = rospy.ServiceProxy(self.stt_service, Trigger)
            resp = svc(TriggerRequest())
            if resp.success and resp.message.strip():
                return resp.message.strip()
            return None
        except Exception as e:
            rospy.logerr('[GPSRCommandParser] STT error: %s', e)
            return None

    # ------------------------------------------------------------------
    def _parse(self, command):
        """
        Call Ollama via VoicebotPrompt service.
        Returns parsed dict or None.
        """
        prompt = self.SYSTEM_PROMPT_TEMPLATE.format(
            locations=', '.join(KNOWN_LOCATIONS),
            objects=', '.join(KNOWN_OBJECTS),
            command=command,
        )

        try:
            from hsr_task_sm.srv import VoicebotPrompt
            rospy.wait_for_service(self.llm_service, timeout=10.0)
            proxy = rospy.ServiceProxy(self.llm_service, VoicebotPrompt)
            result = proxy(prompt)
            response_text = result.response.strip()
        except Exception as e:
            rospy.logerr('[GPSRCommandParser] LLM service error: %s', e)
            return None

        rospy.loginfo('[GPSRCommandParser] LLM raw response: %s', response_text)

        # Extract first JSON object from response (LLM may add extra text)
        match = re.search(r'\{.*?\}', response_text, re.DOTALL)
        if not match:
            rospy.logerr('[GPSRCommandParser] No JSON in response: %s', response_text)
            return None

        try:
            return json.loads(match.group())
        except json.JSONDecodeError as e:
            rospy.logerr('[GPSRCommandParser] JSON parse error: %s', e)
            return None

    # ------------------------------------------------------------------
    def execute(self, userdata):
        # 1. Listen
        self._say("I am listening. Please give me a command.")
        text = self._listen()

        if not text:
            rospy.logwarn('[GPSRCommandParser] No speech detected')
            if self.retry_count < self.retries:
                self.retry_count += 1
                self._say("I did not catch that. Please repeat.")
                text = self._listen()

        if not text:
            userdata.recognized_text = ''
            userdata.parsed_action = ''
            userdata.parsed_target = ''
            userdata.parsed_location = ''
            userdata.parsed_target_location = ''
            userdata.command_description = ''
            return 'failed'

        userdata.recognized_text = text
        rospy.loginfo('[GPSRCommandParser] Heard: %s', text)
        self._say(f"I heard: {text}. Let me understand the command.")

        # 2. Parse
        parsed = self._parse(text)
        if parsed is None:
            rospy.logerr('[GPSRCommandParser] Parsing failed')
            return 'failed'

        # 3. Extract fields
        action = parsed.get('action', '').lower().strip().replace(' ', '_')
        target = parsed.get('target', '')
        location = parsed.get('location', '')
        target_location = parsed.get('target_location', '')
        description = parsed.get('description', text)

        if action not in VALID_ACTIONS:
            rospy.logwarn('[GPSRCommandParser] Unrecognised action: "%s"', action)
            return 'failed'

        # 4. Write to userdata
        userdata.parsed_action = action
        userdata.parsed_target = target
        userdata.parsed_location = location
        userdata.parsed_target_location = target_location
        userdata.command_description = description

        rospy.loginfo(
            '[GPSRCommandParser] action=%s target=%s location=%s target_location=%s',
            action, target, location, target_location
        )

        # Reset retry count for next call
        self.retry_count = 0
        return action
