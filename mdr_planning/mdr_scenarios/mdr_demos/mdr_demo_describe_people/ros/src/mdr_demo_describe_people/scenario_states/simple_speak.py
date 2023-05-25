# import speech_recognition as sr
# import pyttsx3
# import re



# def message_to_floornumber(message):
#     """
#     Finds the patterns in the message and returns the floor number in the form of a list.

#     Args:
#         message (string): The message that needs to interpreted.

#     Returns:
#         list: The list of all floor numbers mentioned in a sentence.
#     """
    
#     message = message.lower()
# #     print(message)
#     number_dict = {
#         "first": 1, "1st": 1,
#         "second": 2, "2nd": 2,
#         "third": 3, "3rd": 3,
#         "fourth": 4, "4th": 4,
#         "fifth": 5, "5th": 5,
#         "sixth": 6, "6th": 6,
#         "seventh": 7, "7th": 7,
#         "eighth": 8, "8th": 8,
#         "ninth": 9, "9th": 9,
#         "tenth": 10, "10th": 10,
#         "ground": 'E', "emergency": 'B',
#         "bell": 'B', "yellow": 'B',
#         "golden": 'B', "open": 'O',
#         "close": 'C', "windex bottle": 'bottle',
#         "shirt": 'shirt', "pringles can": 'pringles',
#         "pringles": 'pringles', "chips": 'pringles',
#         "crisps": 'pringles', "cleaner": 'bottle',
#         "spatula": 'spatula', "campbell soup": 'soup'
#         }
#     pattern = r'\b(?:first|second|third|fourth|fifth|sixth|seventh|eighth|ninth|tenth\
#                     |1st|2nd|3rd|4th|5th|6th|7th|8th|9th|10th|ground) floor\b'
#     pattern_emergency = r'\b(?:emergency|bell|yellow|golden) button\b'
#     pattern_door = r'\b(?:open|close) the doors\b'
    
#     pattern_comp = r'\b(?:windex bottle|shirt|pringles can|spatula|campbell soup)\b'
    
#     floors = re.findall(pattern, message)
#     floors_emergency = re.findall(pattern_emergency, message)
#     floors_door = re.findall(pattern_door, message)
    
#     floors_comp = re.findall(pattern_comp, message)
    
#     if len(floors) != 0:
#         floors = [number_dict[floor.split()[0]] for floor in floors]
#         return floors
    
#     if len(floors_emergency) != 0:
#         floors_emergency = [number_dict[floor.split()[0]] for floor in floors_emergency]
#         return floors_emergency
    
#     if len(floors_door) != 0:
#         floors_door = [number_dict[floor.split()[0]] for floor in floors_door]
#         return floors_door
    
#     if len(floors_comp) != 0:
#         floors_comp = [number_dict[floor.split()[0]] for floor in floors_comp]
#         return floors_comp

# # windex bottle, spatula, pringles can, t-shirt, campbell-soup

# def SpeakText(command):
#     # Initialize the engine
#     engine = pyttsx3.init()
#     engine.say(command)
#     engine.runAndWait()
    

# def Speech_generator():
#     # Loop infinitely for user to
#     # speak
#     initial_sentence = "Hello! Which floor do you want to go on?"
#     interrupt_sentence_1 = "We are on the same floor."
#     interrupt_sentence_2 = "I do not recognize the button you mentioned, please select the specific buttons"
#     interrupt_sentence_3 = "Please, initiate the process again."
    
#     r = sr.Recognizer()
    
#     while(1):
#         command_delay = 0
#         # Exception handling to handle
#         # exceptions at the runtime
#         try:

#             # use the microphone as source for input.
#             with sr.Microphone() as source2:

#                 # wait for a second to let the recognizer
#                 # adjust the energy threshold based on
#                 # the surrounding noise level
#                 r.adjust_for_ambient_noise(source2, duration=0.2)

#                 #listens for the user's input
#                 audio2 = r.listen(source2)

#                 # Using google to recognize audio
#                 MyText = r.recognize_google(audio2)
#                 MyText = MyText.lower()
                
#                 if MyText == "lucy":
#                     SpeakText(initial_sentence)
#                     while(1):   
                        
#                         if command_delay >= 3:
#                             SpeakText(interrupt_sentence_3)
#                             break

#                         # Exception handling to handle
#                         # exceptions at the runtime
#                         try:

#                             # use the microphone as source for input.
#                             with sr.Microphone() as source2:

#                                 # wait for a second to let the recognizer
#                                 # adjust the energy threshold based on
#                                 # the surrounding noise level
#                                 r.adjust_for_ambient_noise(source2, duration=0.2)

#                                 #listens for the user's input
#                                 audio2 = r.listen(source2)

#                                 # Using google to recognize audio
#                                 MyText = r.recognize_google(audio2)
#                                 MyText = MyText.lower()

#                                 # Using The floor number function
#                                 floor_total = message_to_floornumber(MyText)
#                                 print(f"Floor Levels = {floor_total}")
                                
#                                 floor_sentence = f"Lets go to the floor {str(floor_total[0])}"
                                
#                                 print("Did you say ",MyText)
#                                 SpeakText(MyText)
#                                 SpeakText(floor_sentence)
                                
#                                 break

#                         except sr.RequestError as e:
#                             print("Could not request results; {0}".format(e))

#                         except sr.UnknownValueError:
#                             print("unknown error occurred")
#                             command_delay += 1
                    

#         except sr.RequestError as e:
#             print("Could not request results; {0}".format(e))

#         except sr.UnknownValueError:
#             print("unknown error occurred")

# Speech_generator()

import speech_recognition as sr

# Create a recognizer object
r = sr.Recognizer()

# Use the default microphone as the audio source
with sr.Microphone() as source:
    # Adjust the microphone energy threshold for ambient noise levels
    r.adjust_for_ambient_noise(source)
    
    # Prompt the user to speak
    print("Speak now...")
    
    # Record the audio from the microphone
    audio = r.listen(source)

# Perform speech recognition on the audio
text = r.recognize_google(audio)

# Print the transcribed text
print("Transcript: " + text)
