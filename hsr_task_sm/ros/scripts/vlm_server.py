#!/usr/bin/env python3
"""
VLM (Vision Language Model) Query Service

Runs on the slave laptop. Robot sends an image + query type and receives a
single constrained-vocabulary answer. Uses llava (or any vision model) via Ollama.

Slave Laptop Setup:
    1. Install Ollama: curl -fsSL https://ollama.com/install.sh | sh
    2. Pull a vision model: ollama pull llava   (or llava:13b for better accuracy)
    3. Install Python deps: pip install ollama
    4. Set ROS env:
         export ROS_MASTER_URI=http://192.168.50.44:11311
         export ROS_IP=<laptop_ip>
    5. Run: rosrun hsr_task_sm vlm_server.py

ROS Interface:
    Service: /vlm/query  (hsr_task_sm/VLMQuery)

Supported query types and their constrained answers:
    empty_seat      -> "left" | "right" | "both" | "none"
    door_state      -> "open" | "closed"
    shelf_placement -> "top" | "middle" | "bottom"
"""

import base64
import rospy

try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

from hsr_task_sm.srv import VLMQuery, VLMQueryResponse


# ---------------------------------------------------------------------------
# Per-query prompts — each returns ONLY words from the allowed set
# ---------------------------------------------------------------------------

QUERY_CONFIGS = {
    'empty_seat': {
        'allowed': ['left', 'right', 'both', 'none'],
        'prompt': (
            "Look at this image. There is a sofa or seating area. "
            "Which seats are empty and available for a guest to sit?\n"
            "Answer with EXACTLY ONE word from: left, right, both, none.\n"
            "- 'left'  = only the left seat is empty\n"
            "- 'right' = only the right seat is empty\n"
            "- 'both'  = both seats are empty\n"
            "- 'none'  = no empty seats\n"
            "Reply with only the single word. No explanation."
        ),
    },
    'door_state': {
        'allowed': ['open', 'closed'],
        'prompt': (
            "Look at this image. Is the door (washing machine door, dishwasher door, "
            "or any door visible) open or closed?\n"
            "Answer with EXACTLY ONE word: open or closed.\n"
            "Reply with only the single word. No explanation."
        ),
    },
    'shelf_placement': {
        'allowed': ['top', 'middle', 'bottom'],
        'prompt': (
            "Look at this image of a shelf or cabinet. "
            "I am holding a {context} and need to place it on the most appropriate shelf.\n"
            "Answer with EXACTLY ONE word: top, middle, or bottom.\n"
            "- 'top'    = top shelf (light, small, or fragile items)\n"
            "- 'middle' = middle shelf (general everyday items)\n"
            "- 'bottom' = bottom shelf (heavy, large, or food items)\n"
            "Reply with only the single word. No explanation."
        ),
    },
}


class VLMServer:

    def __init__(self):
        rospy.init_node('vlm_server', anonymous=False)

        self.model = rospy.get_param('~model', 'llava')
        self.temperature = rospy.get_param('~temperature', 0.05)

        if not OLLAMA_AVAILABLE:
            rospy.logerr('[VLMServer] ollama package not installed. Run: pip install ollama')
        else:
            rospy.loginfo('[VLMServer] Using ollama model: %s', self.model)

        self.service = rospy.Service('/vlm/query', VLMQuery, self._handle_query)
        rospy.loginfo('[VLMServer] Ready on /vlm/query')

    # ------------------------------------------------------------------

    def _handle_query(self, req):
        if not OLLAMA_AVAILABLE:
            return VLMQueryResponse(answer='', success=False, reason='ollama not installed')

        query_type = req.query_type.strip().lower()
        if query_type not in QUERY_CONFIGS:
            known = list(QUERY_CONFIGS.keys())
            return VLMQueryResponse(
                answer='', success=False,
                reason=f'Unknown query_type "{query_type}". Known: {known}'
            )

        if not req.image_base64:
            return VLMQueryResponse(answer='', success=False, reason='image_base64 is empty')

        cfg = QUERY_CONFIGS[query_type]
        prompt = cfg['prompt'].format(context=req.context or 'object')
        allowed = cfg['allowed']

        rospy.loginfo('[VLMServer] query_type=%s  context="%s"', query_type, req.context)

        try:
            response = ollama.generate(
                model=self.model,
                prompt=prompt,
                images=[req.image_base64],
                options={'temperature': self.temperature},
            )
            raw = response.get('response', '').strip().lower()
            rospy.loginfo('[VLMServer] raw response: "%s"', raw)
        except Exception as e:
            rospy.logerr('[VLMServer] ollama error: %s', e)
            return VLMQueryResponse(answer='', success=False, reason=str(e))

        # Pick the first allowed word found in the response
        answer = self._extract_answer(raw, allowed)
        if answer:
            rospy.loginfo('[VLMServer] answer: %s', answer)
            return VLMQueryResponse(answer=answer, success=True, reason=raw)
        else:
            rospy.logwarn('[VLMServer] Could not extract valid answer from: "%s"', raw)
            return VLMQueryResponse(answer='', success=False,
                                    reason=f'No valid answer in: "{raw}"')

    def _extract_answer(self, text, allowed):
        """Return first allowed word found in text, else None."""
        for word in allowed:
            if word in text.split() or text == word:
                return word
        # Fallback: substring match
        for word in allowed:
            if word in text:
                return word
        return None

    def run(self):
        rospy.spin()


def main():
    try:
        VLMServer().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
