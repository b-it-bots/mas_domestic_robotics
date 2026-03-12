#!/usr/bin/env python3
"""
Pick and Place Challenge States
RoboCup@Home 2026

ClassifyObject:
  Reads grasped_object name from userdata, calls Ollama to decide
  where to put it: dishwasher | trash | cabinet | breakfast | failed

  This drives the tidy-up loop in the YAML state machine.
"""

import re
import rospy
import smach
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest


# ---------------------------------------------------------------------------
# Configurable object lists (update to match your TC object list)
# ---------------------------------------------------------------------------
DISHWASHER_EXAMPLES = [
    'plate', 'bowl', 'mug', 'cup', 'glass',
    'fork', 'knife', 'spoon', 'cutlery', 'utensil', 'chopstick',
]

TRASH_EXAMPLES = [
    'napkin', 'paper', 'wrapper', 'tissue', 'packaging',
    'banana_peel', 'empty_can', 'bottle_empty', 'trash', 'garbage',
]

CABINET_EXAMPLES = [
    'red_bull', 'pringles', 'rubiks_cube', 'colgate', 'toothpaste',
    'book', 'toy', 'snack', 'can', 'bottle', 'cereal_box',
]

BREAKFAST_EXAMPLES = [
    'cereal', 'milk', 'orange_juice', 'milk_carton', 'cereal_box',
    'tablespoon', 'breakfast_bowl', 'breakfast_spoon',
]

VALID_CATEGORIES = ['dishwasher', 'trash', 'cabinet', 'breakfast']


class ClassifyObject(smach.State):
    """
    Classify a held object into a placement category using Ollama LLM.

    Reads grasped_object from userdata (set by PickObject state).
    Returns a SMACH outcome that directly maps to the correct sub-flow
    in the pick-and-place YAML state machine.

    Input Keys:
        grasped_object   Object name/descriptor from PickObject

    Output Keys:
        object_category  One of: dishwasher, trash, cabinet, breakfast

    Outcomes:
        dishwasher | trash | cabinet | breakfast | failed
    """

    PROMPT_TEMPLATE = (
        "[PICK_PLACE_CLASSIFY] You are a domestic robot sorting objects in a kitchen.\n"
        "Given an object name, decide where it should be placed.\n\n"
        "Categories:\n"
        "  dishwasher - tableware needing washing: {dishwasher}\n"
        "  trash      - garbage/waste: {trash}\n"
        "  cabinet    - common items to store: {cabinet}\n"
        "  breakfast  - breakfast items: {breakfast}\n\n"
        "Object: \"{object_name}\"\n\n"
        "Respond with ONLY one word: dishwasher, trash, cabinet, or breakfast"
    )

    def __init__(self, llm_service='/voicebot/prompt'):
        smach.State.__init__(
            self,
            outcomes=VALID_CATEGORIES + ['failed'],
            input_keys=['grasped_object'],
            output_keys=['object_category']
        )
        self.llm_service = llm_service
        self._say_pub = None

    def _say(self, text):
        if self._say_pub is None:
            self._say_pub = rospy.Publisher('/say', String, queue_size=5)
            rospy.sleep(0.2)
        self._say_pub.publish(text)
        rospy.sleep(max(1.0, len(text.split()) * 0.3))

    def _classify_with_llm(self, object_name):
        """Call Ollama to classify the object. Returns category str or None."""
        prompt = self.PROMPT_TEMPLATE.format(
            dishwasher=', '.join(DISHWASHER_EXAMPLES),
            trash=', '.join(TRASH_EXAMPLES),
            cabinet=', '.join(CABINET_EXAMPLES),
            breakfast=', '.join(BREAKFAST_EXAMPLES),
            object_name=object_name,
        )

        try:
            from hsr_task_sm.srv import VoicebotPrompt
            rospy.wait_for_service(self.llm_service, timeout=10.0)
            proxy = rospy.ServiceProxy(self.llm_service, VoicebotPrompt)
            result = proxy(prompt)
            response = result.response.strip().lower()
        except Exception as e:
            rospy.logerr('[ClassifyObject] LLM service error: %s', e)
            return None

        rospy.loginfo('[ClassifyObject] LLM response for "%s": %s', object_name, response)

        # Extract first valid category word from response
        for cat in VALID_CATEGORIES:
            if cat in response:
                return cat

        rospy.logwarn('[ClassifyObject] No valid category in response: %s', response)
        return None

    def _classify_by_name(self, object_name):
        """Fast local fallback — classify by keyword matching."""
        name = object_name.lower().replace('-', '_').replace(' ', '_')

        for keyword in DISHWASHER_EXAMPLES:
            if keyword in name:
                return 'dishwasher'
        for keyword in TRASH_EXAMPLES:
            if keyword in name:
                return 'trash'
        for keyword in BREAKFAST_EXAMPLES:
            if keyword in name:
                return 'breakfast'
        for keyword in CABINET_EXAMPLES:
            if keyword in name:
                return 'cabinet'
        return None

    def execute(self, userdata):
        # Get object name — PickObject may return string or object with .name attribute
        obj = getattr(userdata, 'grasped_object', None)
        if obj is None:
            rospy.logerr('[ClassifyObject] No grasped_object in userdata')
            userdata.object_category = 'failed'
            return 'failed'

        object_name = obj if isinstance(obj, str) else getattr(obj, 'name', str(obj))
        rospy.loginfo('[ClassifyObject] Classifying: %s', object_name)

        # Try local keyword match first (fast, no service call)
        category = self._classify_by_name(object_name)

        if category is None:
            # Fall back to LLM
            rospy.loginfo('[ClassifyObject] Keyword match failed, asking LLM...')
            category = self._classify_with_llm(object_name)

        if category is None:
            rospy.logwarn('[ClassifyObject] Could not classify "%s", defaulting to cabinet', object_name)
            category = 'cabinet'  # safe default — store unknown objects

        rospy.loginfo('[ClassifyObject] "%s" → %s', object_name, category)
        self._say(f"I will put the {object_name} in the {category}.")
        userdata.object_category = category
        return category
