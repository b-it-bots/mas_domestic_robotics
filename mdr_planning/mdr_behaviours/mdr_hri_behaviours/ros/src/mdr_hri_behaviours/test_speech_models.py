import speech_recognition as sr
import time

def record_audio(duration=3, mic_id=None):
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_id) as source:
        print("Recording audio...")
        audio_data = r.record(source, duration=duration)
    return audio_data

def test_google(audio_data):
    r = sr.Recognizer()
    print("Recognizing with Google...")
    start_time = time.time()
    text = r.recognize_google(audio_data, language="en-US")
    end_time = time.time()
    google_time = end_time - start_time
    return text, google_time

def test_whisper(audio_data):  # Replace this with actual method for Whisper
    r = sr.Recognizer()
    print("Recognizing with Whisper...")
    start_time = time.time()
    text = r.recognize_whisper(audio_data, language="english")  # Assuming this is the method for Whisper
    end_time = time.time()
    whisper_time = end_time - start_time
    return text, whisper_time

audio_sample = record_audio()

google_text, google_time = test_google(audio_sample)
whisper_text, whisper_time = test_whisper(audio_sample)
print()
print("="*50)
print()
print(f"Google took {google_time:.4f} seconds.")
print(f"Whisper took {whisper_time:.4f} seconds.")
print()
print("="*50)
print()
if google_time < whisper_time:
    print("Google was faster.")
else:
    print("Whisper was faster.")
