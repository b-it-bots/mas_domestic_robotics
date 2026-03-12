#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import ollama
import json
import re
from llm_server.srv import Prompt, PromptResponse


class VoiceBotService:
    def __init__(
        self,
        model_name="llama3.2",
        system_prompt="",
        out_topic="/llm/response",
    ):
        rospy.loginfo("Initializing VoiceBotService...")
        self.ollama_model = model_name
        self.system_prompt = system_prompt
        self.conversation_history = [{"role": "system", "content": self.system_prompt}]
        self.guests = [{"name": None, "drink": None}]

        # ROS publishers
        self.out_pub = rospy.Publisher(out_topic, String, queue_size=10)
        self.mic_control_pub = rospy.Publisher("/condition_record", Bool, queue_size=10)
        self.say_pub = rospy.Publisher("/say", String, queue_size=10)

        # Mic state: True = listening, False = processing
        self.mic_state = True

        # Advertise service
        self.service = rospy.Service("voicebot/prompt", Prompt, self._handle_request)
        rospy.loginfo("VoiceBotService ready. Service: voicebot/prompt")

    # ---------------- Mic helpers ----------------
    def set_mic(self, state: bool):
        self.mic_state = state
        self.mic_control_pub.publish(Bool(data=self.mic_state))

    # ---------------- String helpers ----------------
    def _strip_code_fences(self, s: str) -> str:
        s = s.strip()
        if s.startswith("```"):
            s = re.sub(r"^```[a-zA-Z]*\n?", "", s)
            s = s.rstrip("`").strip()
        return s

    def _safe_get_content(self, resp) -> str:
        if isinstance(resp, dict):
            return (resp.get("message") or {}).get("content") or ""
        if hasattr(resp, "message") and resp.message is not None:
            return getattr(resp.message, "content", "") or ""
        return ""

    # ---------------- Guest extraction ----------------

    def _validate_extraction(self, data, text):
        text_lower = text.lower()

        for g in ["guest1"]:
            name = data[g].get("name")
            drink = data[g].get("drink")

            if name and name.lower() not in text_lower:
                data[g]["name"] = None

            if drink and drink.lower() not in text_lower:
                data[g]["drink"] = None

        return data
    
    def extract_guest_info(self, user_input: str) -> dict:
        extraction_prompt = f"""
            Return ONLY valid JSON (no markdown).

            Schema:
            {{ "guest1": {{"name": null, "drink": null}} }}

            Rules:
            - Extract ONLY information explicitly present in the text.
            - Do NOT guess or infer missing values.
            - If a value is not explicitly spoken, return null.
            - The name or drink MUST appear exactly in the text.

            Text: {user_input}
            """
#         extraction_prompt = f"""
# Return ONLY valid JSON (no markdown, no comments) in exactly this schema:
# {{ "guest1": {{"name": null, "drink": null}}, "guest2": {{"name": null, "drink": null}} }}
# If no valid info is found, return null for both fields.
# Only extract names or drinks explicitly mentioned — do not substitute.
# Text: {user_input}
# """
        data = None
        try:
            resp = ollama.chat(
                model=self.ollama_model,
                messages=[
                    {"role": "system", "content": "You extract structured data. Return only JSON."},
                    {"role": "user", "content": extraction_prompt},
                ],
                stream=False,
            )
            content = self._strip_code_fences(self._safe_get_content(resp))
            data = json.loads(content)
        except Exception:
            data = None


            # ---------------- Validate LLM output ----------------
        text_lower = user_input.lower()

        if isinstance(data, dict):
            for g in ["guest1"]:

                if g not in data or not isinstance(data[g], dict):
                    data[g] = {"name": None, "drink": None}

                name = data[g].get("name")
                drink = data[g].get("drink")

                # Reject hallucinated names
                if name and name.lower() not in text_lower:
                    data[g]["name"] = None

                # Reject hallucinated drinks
                if drink and drink.lower() not in text_lower:
                    data[g]["drink"] = None
        else:
            data = {}

        # Regex fallback
        text = user_input.strip()
        name = None
        m = re.search(r"\bmy name is\s+([A-Za-z][A-Za-z'-]{1,30})\b", text, re.IGNORECASE)
        if m:
            name = m.group(1)
        else:
            m = re.search(r"\b(i[' ]?m|i am)\s+([A-Za-z][A-Za-z'-]{1,30})\b", text, re.IGNORECASE)
            if m:
                name = m.group(2)

        drink = None
        m = re.search(r"\bmy favorite drink is\s+([A-Za-z][A-Za-z0-9' -]{1,40})\b", text, re.IGNORECASE)
        if m:
            drink = m.group(1).strip()
        else:
            m = re.search(r"\bi like (?:to drink )?([A-Za-z][A-Za-z0-9' -]{1,40})\b", text, re.IGNORECASE)
            if m:
                drink = m.group(1).strip()

        # Enforce schema
        if not isinstance(data, dict):
            data = {}
        for g in ["guest1"]:
            if g not in data or not isinstance(data[g], dict):
                data[g] = {"name": None, "drink": None}
            else:
                data[g].setdefault("name", None)
                data[g].setdefault("drink", None)

        if name and not (data.get("guest1") or {}).get("name"):
            data["guest1"]["name"] = name
        if drink and not (data.get("guest1") or {}).get("drink"):
            data["guest1"]["drink"] = drink

        return data

    def _store_guest_data(self, name=None, drink=None):
        if name and name.strip().lower() == "lucy":
            return
        if name:
            if self.guests[0]["name"] is None:
                self.guests[0]["name"] = name
            # elif (
            #     self.guests[1]["name"] is None
            #     and self.guests[0]["name"].lower() != name.lower()
            # ):
            #     self.guests[1]["name"] = name
        if drink:
            if name and self.guests[0]["name"] == name:
                self.guests[0]["drink"] = drink
            # elif name and self.guests[1]["name"] == name:
            #     self.guests[1]["drink"] = drink
            elif self.guests[0]["drink"] is None:
                self.guests[0]["drink"] = drink
            # elif self.guests[1]["drink"] is None:
                # self.guests[1]["drink"] = drink


    def reset_llm_memory(self):
        rospy.loginfo("🔄 Resetting LLM memory for next guest")
        self.conversation_history = [
            {"role": "system", "content": self.system_prompt}
        ]

    # ---------------- LLM ----------------
    def get_llm_response(self, user_input: str) -> str:
        rospy.loginfo("🤖 Thinking...")
        self.conversation_history.append({"role": "user", "content": user_input})

        memory_context = (
            f"Known guest information:\n"
            f"Guest 1: Name={self.guests[0]['name']}, Drink={self.guests[0]['drink']}\n"
            # f"Guest 2: Name={self.guests[1]['name']}, Drink={self.guests[1]['drink']}\n"
        )
        messages = (
            [{"role": "system", "content": self.system_prompt},
             {"role": "system", "content": memory_context}]
            + self.conversation_history[1:]
        )

        response_text = ""
        try:
            stream = ollama.chat(
                model=self.ollama_model,
                messages=messages,
                stream=True,
                options={"temperature": 0.0},
            )
            for chunk in stream:
                if hasattr(chunk, "message") and chunk.message is not None:
                    content = getattr(chunk.message, "content", None)
                    if content:
                        response_text += content
                        continue
                if isinstance(chunk, dict):
                    msg = chunk.get("message")
                    if isinstance(msg, dict):
                        content = msg.get("content")
                        if content:
                            response_text += content
                        continue
                    content = chunk.get("response")
                    if content:
                        response_text += content
        except Exception as e:
            rospy.logerr(f"❌ LLM error: {e}")
            response_text = "I'm sorry, I encountered an error processing your request."

        self.conversation_history.append({"role": "assistant", "content": response_text})
        return response_text

    # ---------------- Service handler ----------------
    def _handle_request(self, req: Prompt) -> PromptResponse:
        user_text = (req.prompt or "").strip()
        rospy.loginfo(f"👤 Service request: {user_text}")

        if not user_text:
            return PromptResponse(response="No input received.", guests_json="{}")

        # Disable mic while processing
        self.set_mic(False)

        # --- Extract & store guest info ---
        info = self.extract_guest_info(user_text)
        if info:
            if info.get("guest1"):
                self._store_guest_data(
                    name=info["guest1"].get("name"),
                    drink=info["guest1"].get("drink"),
                )
            if info.get("guest2"):
                self._store_guest_data(
                    name=info["guest2"].get("name"),
                    drink=info["guest2"].get("drink"),
                )

        rospy.loginfo(f"🧠 Guest Memory: {self.guests}")

        # # Persist complete guest state to JSON files
        # if self.guests[0]["name"] and self.guests[0]["drink"]:
        #     with open("./person1.json", "w") as f:
        #         json.dump({"guest1": self.guests[0]}, f, indent=4)

        # if self.guests[1]["name"] and self.guests[1]["drink"]:
        #     with open("./person2.json", "w") as f:
        #         json.dump({"guest2": self.guests[1]}, f, indent=4)

        # Build guests JSON payload for the response
        guests_payload = {
            "guest1": self.guests[0]
            # "guest2": self.guests[1],
        }
        guests_json = json.dumps(guests_payload)


        rospy.loginfo(f"📊 Guests JSON: {self.guests}")
        # --- Handle exit commands ---
        if any(word in user_text.lower() for word in ["exit", "quit", "goodbye", "bye"]):
            response_text = "Goodbye! Have a great day!"
            self.out_pub.publish(String(data=response_text))
            rospy.loginfo("Exit command received.")
            self.set_mic(True)
            return PromptResponse(response=response_text, guests_json=guests_json)

        # --- LLM response ---
        response_text = self.get_llm_response(user_text)
        rospy.loginfo(f"💬 Bot: {response_text}")
        self.out_pub.publish(String(data=response_text))

        # Re-enable mic after processing
        self.set_mic(True)

        # reset LLM memory if both guests' info is collected

        if guests_payload["guest1"]["name"] and guests_payload["guest1"]["drink"]:

            rospy.loginfo("✅ Guest information complete. Saving and resetting.")

            # Reset guest storage
            self.guests = [{"name": None, "drink": None}]

            # Reset LLM conversation memory
            self.reset_llm_memory()

        return PromptResponse(response=response_text, guests_json=guests_json)


def main():
    rospy.init_node("voicebot_service", anonymous=False)

    out_topic = rospy.get_param("~out_topic", "/llm/response")
    ollama_model = rospy.get_param("~ollama_model", "llama3.2")
    system_prompt = rospy.get_param(
        "~system_prompt",
        """
You are Lucy, a receptionist greeting guests. You can only greet and ask these questions:
- their names
- their favorite drink
Do not ask any additional questions.
After getting the name and favorite drink, ask the user to stand still to take a picture, so that they could describe them to the other guest.
Keep replies short and natural.
""".strip(),
    )

    VoiceBotService(
        model_name=ollama_model,
        system_prompt=system_prompt,
        out_topic=out_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()