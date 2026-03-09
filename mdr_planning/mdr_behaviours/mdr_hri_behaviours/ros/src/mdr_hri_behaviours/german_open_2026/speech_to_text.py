import speech_recognition as sr
import rospy
from std_msgs.msg import String
import time

class SpeechRecognitionService:
    _instances = {}
    
    @classmethod
    def get_instance(cls, instance_id="default", **kwargs):
        """Get a specific named instance or create it if it doesn't exist"""
        if instance_id not in cls._instances:
            cls._instances[instance_id] = cls(instance_id=instance_id, **kwargs)
        return cls._instances[instance_id]
    
    @classmethod
    def create_new_instance(cls, instance_id=None, **kwargs):
        """Always create a new instance with an optional ID"""
        if instance_id is None:
            instance_id = f"instance_{len(cls._instances)}"
        instance = cls(instance_id=instance_id, **kwargs)
        cls._instances[instance_id] = instance
        return instance
    
    def __init__(self, instance_id="default", device_part_name="Razer Barracuda X", pause_threshold=1.5, 
                 adjust_duration=20):
        self.instance_id = instance_id
        self.device_part_name = device_part_name
        
        # Initialize recognizer
        self.r = sr.Recognizer()
        self.r.pause_threshold = pause_threshold
        
        # Initialize microphone
        self.init_microphone(adjust_duration)
        
        rospy.loginfo(f"Speech Recognition Service instance '{instance_id}' initialized")
        ## For the robot to speak
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)
        
        ##For interest extraction
        self.extracted_interest = None
        self.interest_received = False
        #self.detected_interest=rospy.Subscriber('ollama_detected_interest', String, self.interest_callback)
        #self.interest_input_pub = rospy.Publisher('interest_input', String, queue_size=10)
    
    def init_microphone(self, adjust_duration=20):
        # Initialize microphone
        self.mic_source = sr.Microphone()
        with self.mic_source as source:
            self.r.adjust_for_ambient_noise(source, duration=adjust_duration)
            rospy.loginfo(f"[{self.instance_id}] Microphone is set up and ambient noise level adjusted.")
            
    def say_this(self, message):
        self.say_pub.publish(message)
        rospy.loginfo(f"Saying: {message}")

    def listen_and_transcribe(self):
        rospy.loginfo(f"[{self.instance_id}] Listening...")
        with self.mic_source as source:
            audio = self.r.listen(source, timeout=6)
        try:
            # Directly use AudioData object for recognition
            recognized_text = self.r.recognize_google(audio)
            rospy.loginfo(f"[{self.instance_id}] Recognized Text: {recognized_text}")
            return recognized_text
        except (sr.UnknownValueError, sr.RequestError) as e:
            rospy.loginfo(f"[{self.instance_id}] Speech recognition error: {e}")
            return None
        
    def confirm_response(self, confirmation_not_understood_msg, not_heard_msg,try_again_msg, max_confirmation_attempts = 3):
        confirmation_attempts = 0
        confirmed = False
        while confirmation_attempts < max_confirmation_attempts:
                confirmation_attempts += 1
                confirmation_response = self.listen_and_transcribe()

                if confirmation_response:
                    confirmation_response_lower = confirmation_response.lower()
                    if any(word in confirmation_response_lower for word in ["yes", "yeah", "yep","i have placed","it is in your hand","done","confirmed"]):
                        confirmed = True
                        break
                    elif any(word in confirmation_response_lower for word in ["no", "not", "wrong", "incorrect", "nope","not yet"]):
                        self.say_this(try_again_msg)
                        confirmed = False
                        break
                    else:
                        self.say_this(confirmation_not_understood_msg)
                else:
                    self.say_this(not_heard_msg)
                    
        return confirmed
        
    def get_and_confirm_input(self, initial_prompt, validation_func, confirmation_format=None, 
                         failed_validation_msg=None, not_heard_msg=None,
                         confirmation_not_understood_msg=None, try_again_msg=None,
                         success_msg_format=None, max_attempts=3):
        """
        Prompts for input, validates it, then asks for confirmation before accepting.
        
        Args:
            initial_prompt: Initial question to ask the user
            validation_func: Function to validate the initial response
            confirmation_format: Format string for confirmation (default: "I heard {value}. Is that correct?")
            failed_validation_msg: Message when validation fails (default: "I could not understand that")
            not_heard_msg: Message when nothing is heard (default: "I did not hear you")
            confirmation_not_understood_msg: Message when confirmation response isn't clear
            try_again_msg: Message to say before trying again after rejection (default: "Let's try again")
            success_msg_format: Format string for success message (default: "Great!")
            max_attempts: Maximum number of attempts (None for unlimited)
            
        Returns:
            The validated and confirmed value, or None if max attempts reached
        """
        # Set default messages
        if confirmation_format is None:
            confirmation_format = "I heard {value}. Is that correct,please say yes or no?"
        if failed_validation_msg is None:
            failed_validation_msg = "I could not understand that, could you please repeat?"
        if not_heard_msg is None:
            not_heard_msg = "I did not hear you, could you please speak louder?"
        if confirmation_not_understood_msg is None:
            confirmation_not_understood_msg = "I didn't catch that. Please say yes or no."
        if try_again_msg is None:
            try_again_msg = "I am sorry, let's try again."
        if success_msg_format is None:
            success_msg_format = "Great!"
        rospy.loginfo("Inside speech_to_text function, starting with questions")
        attempts = 0
        while attempts <= max_attempts:
            attempts += 1
            rospy.loginfo(f"[{self.instance_id}] Attempt {attempts}" + 
                         (f" of {max_attempts}" if max_attempts else ""))

            self.say_this(initial_prompt)

            response = self.listen_and_transcribe()
            rospy.loginfo(f"[{self.instance_id}] User said: {response}")

            if response:
                validated_value = validation_func(response)
                rospy.loginfo(f"Name mapped: {validated_value}")
                if validated_value:
                    confirmation_prompt = confirmation_format.format(value=validated_value)
                    self.say_this(confirmation_prompt)
                    not_heard_msg_confirmation = "I did not hear you, could you please say yes or no louder?"
                    confirmed = self.confirm_response(confirmation_not_understood_msg, not_heard_msg_confirmation, try_again_msg)

                    if confirmed:
                        success_msg = success_msg_format.format(value=validated_value)
                        self.say_this(success_msg)
                        return validated_value
                else:
                    self.say_this(failed_validation_msg)
            else:
                self.say_this(not_heard_msg)

            # rospy.sleep(2) 

        rospy.loginfo(f"[{self.instance_id}] Max attempts ({max_attempts}) reached without success")
        return None
    
