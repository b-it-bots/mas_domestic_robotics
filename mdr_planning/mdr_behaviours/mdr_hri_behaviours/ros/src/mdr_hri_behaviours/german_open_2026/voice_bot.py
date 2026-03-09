"""
Real-time Agentic Voice Bot
Uses Whisper for speech recognition and Ollama for LLM responses
- Adds a SYSTEM PROMPT to Ollama (role="system") for consistent behavior
"""

import pyaudio
import wave
import whisper
import ollama
import numpy as np
from gtts import gTTS
import tempfile
import os
import queue
import json
import re


class VoiceBot:
    def __init__(
        self,
        model_name="llama3.2",
        whisper_model="base",
        system_prompt=None,
    ):
        """
        Initialize the voice bot

        Args:
            model_name: Ollama model to use (e.g., 'llama3.2', 'mistral', 'phi3')
            whisper_model: Whisper model size ('tiny', 'base', 'small', 'medium', 'large')
            system_prompt: System prompt to steer the assistant behavior
        """
        print("Initializing Voice Bot...")

        # Audio settings
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100  # 16000 is common for speech; keeping your original
        self.SILENCE_THRESHOLD = 500  # Adjust based on your microphone
        self.SILENCE_DURATION = 2.0  # Seconds of silence to stop recording

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Load Whisper model
        print(f"Loading Whisper model: {whisper_model}...")
        self.whisper_model = whisper.load_model(whisper_model)

        # Set Ollama model
        self.ollama_model = model_name

        self.system_prompt = system_prompt

        # Store guest data
        self.guests = [
            {"name": None, "drink": None},
            {"name": None, "drink": None},
        ]


        # ✅ Conversation history starts with system message
        self.conversation_history = [
            {"role": "system", "content": self.system_prompt}
        ]

        # Audio queue (kept from your original; not required for current loop)
        self.audio_queue = queue.Queue()

        print("Voice Bot initialized successfully!")
        print(f"Using Whisper model: {whisper_model}")
        print(f"Using Ollama model: {model_name}")
        print("System prompt is set.")
    

    def get_rms(self, data):
        """Calculate RMS (root mean square) of audio data"""
        try:
            audio_data = np.frombuffer(data, dtype=np.int16)
            if len(audio_data) == 0:
                return 0.0

            audio_float = audio_data.astype(np.float64)
            mean_square = np.mean(audio_float ** 2)

            if mean_square < 0 or np.isnan(mean_square):
                return 0.0

            rms = np.sqrt(mean_square)

            if np.isnan(rms) or np.isinf(rms):
                return 0.0

            return float(rms)

        except Exception:
            return 0.0
            
    def _strip_code_fences(self, s: str) -> str:
        s = s.strip()
        if s.startswith("```"):
            s = re.sub(r"^```[a-zA-Z]*\n?", "", s)
            s = s.rstrip("`").strip()
        return s

    def _safe_get_content(self, resp):
        # Handles dict OR pydantic-style objects
        if isinstance(resp, dict):
            return (resp.get("message") or {}).get("content") or ""
        if hasattr(resp, "message") and resp.message is not None:
            return getattr(resp.message, "content", "") or ""
        return ""

    def extract_guest_info(self, user_input: str):
        """
        Returns dict:
        {"guest1": {"name": str|None, "drink": str|None}, "guest2": {...}}
        """
        extraction_prompt = f"""
    Return ONLY valid JSON (no markdown, no comments) in exactly this schema:
    {{
    "guest1": {{"name": null, "drink": null}},
    "guest2": {{"name": null, "drink": null}}
    }}

    Extract up to two guests' names and favorite drinks from the text.
    If missing, keep null.

    Text: {user_input}
    """

        data = None
        try:
            resp = ollama.chat(
                model=self.ollama_model,
                messages=[
                    {"role": "system", "content": "You extract structured data. Return only JSON."},
                    {"role": "user", "content": extraction_prompt},
                ],
                stream=False
            )
            content = self._strip_code_fences(self._safe_get_content(resp))
            data = json.loads(content)
        except Exception:
            data = None

        # --- Regex fallback for common patterns (fills gaps) ---
        text = user_input.strip()

        # name: "my name is X" or "I'm X" or "I am X"
        name = None
        m = re.search(r"\bmy name is\s+([A-Za-z][A-Za-z'-]{1,30})\b", text, re.IGNORECASE)
        if m:
            name = m.group(1)
        else:
            m = re.search(r"\b(i[' ]?m|i am)\s+([A-Za-z][A-Za-z'-]{1,30})\b", text, re.IGNORECASE)
            if m:
                name = m.group(2)

        # drink: "I like X" / "my favorite drink is X"
        drink = None
        m = re.search(r"\bmy favorite drink is\s+([A-Za-z][A-Za-z0-9' -]{1,40})\b", text, re.IGNORECASE)
        if m:
            drink = m.group(1).strip()
        else:
            m = re.search(r"\bi like\s+([A-Za-z][A-Za-z0-9' -]{1,40})\b", text, re.IGNORECASE)
            if m:
                drink = m.group(1).strip()

        # Merge fallback into LLM extraction
        if data is None:
            data = {"guest1": {"name": None, "drink": None}, "guest2": {"name": None, "drink": None}}

        if name and not (data.get("guest1") or {}).get("name"):
            data["guest1"]["name"] = name
        if drink and not (data.get("guest1") or {}).get("drink"):
            data["guest1"]["drink"] = drink

        return data


    def record_audio(self):
        """Record audio from microphone with silence detection"""
        print("\n🎤 Listening... (speak now)")

        stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
        )

        frames = []
        silent_chunks = 0
        started_speaking = False
        max_silent_chunks = int(self.SILENCE_DURATION * self.RATE / self.CHUNK)

        try:
            while True:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)

                rms = self.get_rms(data)

                # Detect speech start
                if rms > self.SILENCE_THRESHOLD:
                    started_speaking = True
                    silent_chunks = 0
                elif started_speaking:
                    silent_chunks += 1

                # Stop if silence detected after speaking
                if started_speaking and silent_chunks > max_silent_chunks:
                    print("✓ Finished recording")
                    break

                # Safety limit: 30 seconds max
                if len(frames) > self.RATE * 30 / self.CHUNK:
                    print("⚠ Max recording time reached")
                    break

        finally:
            stream.stop_stream()
            stream.close()

        return frames

    def save_audio(self, frames, filename):
        """Save recorded audio to WAV file"""
        wf = wave.open(filename, "wb")
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b"".join(frames))
        wf.close()

    def transcribe_audio(self, audio_file):
        """Transcribe audio using Whisper"""
        print("🔄 Transcribing...")
        result = self.whisper_model.transcribe(audio_file)
        return result.get("text", "").strip()

    def get_llm_response(self, user_input):
        """Get response from Ollama LLM"""
        print("🤖 Thinking...")

        self.conversation_history.append({"role": "user", "content": user_input})

        response_text = ""
        memory_context = f"""
        Known guest information:
        Guest 1: Name={self.guests[0]['name']}, Drink={self.guests[0]['drink']}
        Guest 2: Name={self.guests[1]['name']}, Drink={self.guests[1]['drink']}
        """

        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "system", "content": memory_context}
        ] + self.conversation_history[1:]  # skip original system 
        try:
            stream = ollama.chat(
                model=self.ollama_model,
                messages=self.conversation_history,
                stream=True,
            )

            print("💬 Response: ", end="", flush=True)

            for chunk in stream:
                # ✅ Your environment: chunk is an object (not dict)
                if hasattr(chunk, "message") and chunk.message is not None:
                    content = getattr(chunk.message, "content", None)
                    if content:
                        response_text += content
                        print(content, end="", flush=True)
                    continue

                # ✅ Other environments: chunk might be a dict
                if isinstance(chunk, dict):
                    msg = chunk.get("message")
                    if isinstance(msg, dict):
                        content = msg.get("content")
                        if content:
                            response_text += content
                            print(content, end="", flush=True)
                        continue

                    content = chunk.get("response")
                    if content:
                        response_text += content
                        print(content, end="", flush=True)
                        continue

                    if chunk.get("error"):
                        print("\n❌ Ollama error:", chunk["error"])
                        break

            print()

        except Exception as e:
            print(f"\n❌ Error getting LLM response: {e}")
            response_text = "I'm sorry, I encountered an error processing your request."

        self.conversation_history.append({"role": "assistant", "content": response_text})
        return response_text


    def run(self):
        """Main loop for the voice bot"""
        print("\n" + "=" * 60)
        print("🤖 VOICE BOT STARTED")
        print("=" * 60)
        print("Press Ctrl+C to exit")
        print("Speak after you see '🎤 Listening...'")
        print("=" * 60 + "\n")

        try:
            while True:
                # Record audio
                audio_frames = self.record_audio()

                # Save to temporary file
                with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as fp:
                    temp_audio_file = fp.name

                self.save_audio(audio_frames, temp_audio_file)

                # Transcribe
                user_text = self.transcribe_audio(temp_audio_file)

                # Remove temp wav
                try:
                    os.remove(temp_audio_file)
                except OSError:
                    pass

                if not user_text:
                    print("❌ No speech detected, try again...")
                    continue

                print(f"👤 You said: {user_text}")

                info = self.extract_guest_info(user_text)

                if info:
                    if info.get("guest1"):
                        if info["guest1"].get("name"):
                            self.guests[0]["name"] = info["guest1"]["name"]
                        if info["guest1"].get("drink"):
                            self.guests[0]["drink"] = info["guest1"]["drink"]

                    if info.get("guest2"):
                        if info["guest2"].get("name"):
                            self.guests[1]["name"] = info["guest2"]["name"]
                        if info["guest2"].get("drink"):
                            self.guests[1]["drink"] = info["guest2"]["drink"]

                print("🧠 Current Guest Memory:", self.guests)

                # Exit command
                if any(word in user_text.lower() for word in ["exit", "quit", "goodbye", "bye"]):
                    response = "Goodbye! Have a great day!"
                    print(f"💬 Bot: {response}")
                    # self.speak(response)
                    break

                # Get LLM response
                response = self.get_llm_response(user_text)

                print("\n" + "-" * 60 + "\n")
                print(f"💬 Bot: {response}")

                # Speak response
                # self.speak(response)

                print("\n" + "-" * 60 + "\n")

        except KeyboardInterrupt:
            print("\n\n👋 Voice Bot stopped by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        try:
            self.audio.terminate()
        except Exception:
            pass
        print("✓ Cleanup complete")


def main():
    """Main function to run the voice bot"""

    # Customize these parameters
    OLLAMA_MODEL = "llama3.2"
    WHISPER_MODEL = "medium.en"

    # Optional: customize system prompt here
    SYSTEM_PROMPT = """
                You are  lucy a receptionist greeting guests you can only greet and ask questions but cannot do any action like serving a drink etc.
                Start with a warm welcome and always ask the guest questions required.
                Ask the guests for:
                - their names
                - their favorite drink
                Do not ask any additional questions other than the name and favorite drink.
                After getting the name and favorite drink as the user to follow you to the seating area and dont follow up with any other question.
                Keep the reply short and natural.
                """

    bot = VoiceBot(
        model_name=OLLAMA_MODEL,
        whisper_model=WHISPER_MODEL,
        system_prompt=SYSTEM_PROMPT,
    )
    bot.run()


if __name__ == "__main__":
    main()
