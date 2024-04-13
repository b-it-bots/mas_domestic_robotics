import rospy
import speech_recognition as sr
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import String  # ROS standard message

class MultiGuestIntroductionTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'multi_guest_introduction_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['persons_record'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5  # Adjust the value as needed

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def execute(self, userdata):
        rospy.loginfo("Initiating multi-guest introduction...")

        # Determine if the current guest is the only guest or one of multiple
        if len(userdata.persons_record) == 1:
            # Handling the first guest
            new_guest = userdata.persons_record[0]
            new_guest_name = new_guest["guest_name"]
            new_guest_drink = new_guest["favorite_drink"]
            rospy.loginfo("Only one guest, making initial introduction...")
            intro_phrase = f"{new_guest_name}, please have a seat on the couch."
            self.say_this(intro_phrase)
            rospy.sleep(2)
        else:
            # Handling subsequent guests
            new_guest = userdata.persons_record[-1]
            new_guest_name = new_guest["guest_name"]
            new_guest_drink = new_guest["favorite_drink"]
            seating_phrase = f"Hello everyone, please meet {new_guest_name} who likes to drink {new_guest_drink}. {new_guest_name}, please have a seat on the couch."
            self.say_this(seating_phrase)
            rospy.sleep(2)

            # Introduce all other guests to the new guest
            intro_phrases = []
            for previous_guest in userdata.persons_record[:-1]:
                guest_name = previous_guest["guest_name"]
                guest_drink = previous_guest["favorite_drink"]
                intro_phrases.append(f"meet {guest_name}, who likes {guest_drink}")
                
            if intro_phrases:
                introduction_sentence = f"{new_guest_name}, please " + ", ".join(intro_phrases) + "."
                self.say_this(introduction_sentence)      

        """if len(userdata.persons_record) < 2:
            rospy.loginfo("Not enough guests for full introductions.")
            return 'succeeded'
        new_guest = userdata.persons_record[-1]
        new_guest_name = new_guest["guest_name"]
        new_guest_drink = new_guest["favorite_drink"]

        # Seat the new guest first with a welcoming phrase
        seating_phrase = f"Hello everyone, please meet {new_guest_name} who likes to drink {new_guest_drink}."
        self.say_this(seating_phrase)
        rospy.sleep(2)  # Give time for the new guest to be seated

        # Introduce all other guests to the new guest
        intro_phrases = []
        for previous_guest in userdata.persons_record[:-1]:
            guest_name = previous_guest["guest_name"]
            guest_drink = previous_guest["favorite_drink"]
            intro_phrases.append(f"meet {guest_name}, who likes {guest_drink}")
            #rospy.sleep(1)  # Pause between guest introductions to allow for better processing

        introduction_sentence = f"{new_guest_name}, please " + ", ".join(intro_phrases) + "."
        self.say_this(introduction_sentence)"""

        rospy.sleep(2)
        # Final courtesy message after all introductions
        self.say_this(f"It was a pleasure assisting you, {new_guest_name}. Enjoy your time!")
        rospy.sleep(1)  # Give a moment for the message to resonate
        
        rospy.loginfo("All introductions completed successfully.")
        return 'succeeded'

# ROS setup and ScenarioStateBase should be defined correctly in the actual robot environment.
