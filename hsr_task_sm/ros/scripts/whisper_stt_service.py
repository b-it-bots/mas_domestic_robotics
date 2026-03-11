#!/usr/bin/env python3
"""
Whisper Speech-to-Text Service

This service runs on the slave laptop and provides speech recognition
using OpenAI's Whisper model. It records audio from the microphone,
transcribes it, and returns the text.

Slave Laptop Setup:
    1. Install dependencies:
       pip install openai-whisper pyaudio numpy
    
    2. Set ROS_MASTER_URI to point to robot:
       export ROS_MASTER_URI=http://<robot_ip>:11311
       export ROS_IP=<slave_laptop_ip>
    
    3. Run this service:
       rosrun hsr_task_sm whisper_stt_service.py

ROS Interface:
    Service: /speech_recognize (std_srvs/Trigger)
    Publisher: /speech/transcript (std_msgs/String)
    Subscriber: /condition_record (std_msgs/Bool) - mic control
"""

import os
import tempfile
import subprocess
import numpy as np
import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

# Try to import whisper, provide helpful error if missing
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    rospy.logwarn("Whisper not installed. Install with: pip install openai-whisper")


class WhisperSTTService:
    """
    Whisper Speech-to-Text Service
    
    Parameters:
        ~model_size (str): Whisper model size - tiny, base, small, medium, large
                          Use .en suffix for English-only models (faster)
        ~record_duration (int): Recording duration in seconds
        ~record_device (str): ALSA device for recording (default: 'default')
        ~language (str): Language code for transcription (default: 'en')
    """
    
    def __init__(self):
        rospy.init_node('whisper_stt_service', anonymous=False)
        
        # Parameters
        self.model_size = rospy.get_param('~model_size', 'base.en')
        self.record_duration = rospy.get_param('~record_duration', 5)
        self.record_device = rospy.get_param('~record_device', 'default')
        self.language = rospy.get_param('~language', 'en')
        self.sample_rate = rospy.get_param('~sample_rate', 16000)
        
        # Mic control state
        self.mic_enabled = True
        
        # Load Whisper model
        if WHISPER_AVAILABLE:
            rospy.loginfo(f"Loading Whisper model: {self.model_size}")
            self.model = whisper.load_model(self.model_size)
            rospy.loginfo("Whisper model loaded successfully")
        else:
            self.model = None
            rospy.logerr("Whisper not available - service will return errors")
        
        # Publishers
        self.transcript_pub = rospy.Publisher('/speech/transcript', String, queue_size=10)
        
        # Subscribers
        self.mic_control_sub = rospy.Subscriber('/condition_record', Bool, self._mic_control_cb)
        
        # Service
        self.service = rospy.Service('speech_recognize', Trigger, self._handle_recognize)
        
        rospy.loginfo("Whisper STT Service ready")
        rospy.loginfo(f"  Model: {self.model_size}")
        rospy.loginfo(f"  Record duration: {self.record_duration}s")
        rospy.loginfo(f"  Language: {self.language}")
    
    def _mic_control_cb(self, msg):
        """Handle mic enable/disable commands."""
        self.mic_enabled = msg.data
        state = "enabled" if self.mic_enabled else "disabled"
        rospy.loginfo(f"Microphone {state}")
    
    def _record_audio(self, output_file):
        """
        Record audio using arecord (ALSA).
        
        Returns:
            bool: True if recording succeeded
        """
        cmd = [
            'arecord',
            '-D', self.record_device,
            '-f', 'S16_LE',
            '-r', str(self.sample_rate),
            '-c', '1',
            '-d', str(self.record_duration),
            output_file
        ]
        
        try:
            rospy.loginfo(f"Recording audio for {self.record_duration} seconds...")
            result = subprocess.run(cmd, capture_output=True, timeout=self.record_duration + 5)
            
            if result.returncode != 0:
                rospy.logerr(f"arecord failed: {result.stderr.decode()}")
                return False
            
            rospy.loginfo("Recording complete")
            return True
            
        except subprocess.TimeoutExpired:
            rospy.logerr("Recording timed out")
            return False
        except Exception as e:
            rospy.logerr(f"Recording error: {e}")
            return False
    
    def _transcribe(self, audio_file):
        """
        Transcribe audio file using Whisper.
        
        Returns:
            str: Transcribed text or None on error
        """
        if not self.model:
            return None
        
        try:
            rospy.loginfo("Transcribing audio...")
            result = self.model.transcribe(
                audio_file,
                language=self.language,
                temperature=0.0,
                no_speech_threshold=0.6
            )
            
            text = result.get('text', '').strip()
            
            if text:
                rospy.loginfo(f"Transcription: {text}")
                return text
            else:
                rospy.logwarn("Empty transcription")
                return None
                
        except Exception as e:
            rospy.logerr(f"Transcription error: {e}")
            return None
    
    def _handle_recognize(self, req):
        """
        Handle speech recognition service request.
        """
        # Check if mic is enabled
        if not self.mic_enabled:
            rospy.logwarn("Microphone is disabled")
            return TriggerResponse(
                success=False,
                message="Microphone disabled"
            )
        
        # Check if Whisper is available
        if not self.model:
            return TriggerResponse(
                success=False,
                message="Whisper model not loaded"
            )
        
        # Create temporary file for audio
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            temp_file = f.name
        
        try:
            # Record audio
            if not self._record_audio(temp_file):
                return TriggerResponse(
                    success=False,
                    message="Recording failed"
                )
            
            # Transcribe
            text = self._transcribe(temp_file)
            
            if text:
                # Publish transcript
                self.transcript_pub.publish(String(data=text))
                
                return TriggerResponse(
                    success=True,
                    message=text
                )
            else:
                return TriggerResponse(
                    success=False,
                    message="No speech recognized"
                )
        
        finally:
            # Cleanup temp file
            if os.path.exists(temp_file):
                os.remove(temp_file)
    
    def run(self):
        """Run the service."""
        rospy.spin()


def main():
    try:
        service = WhisperSTTService()
        service.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
