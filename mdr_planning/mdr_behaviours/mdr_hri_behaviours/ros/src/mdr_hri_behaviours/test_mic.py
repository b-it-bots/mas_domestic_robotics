# #!/usr/bin/env python

# # Import required modules
# import rospy
# import speech_recognition as sr
# from std_msgs.msg import String

# # Create a SpeechRecognition object
# r = sr.Recognizer()

def recognize_speech(device_id=0):
    with sr.Microphone(device_index=device_id) as source:
        print("Listening...")
        audio_data = r.record(source, duration=5)  # record audio from the default microphone for 5 seconds
        print("Recognizing...")
        # using google speech recognition
        text = r.recognize_google(audio_data)
        return text

# def start_node():
#     rospy.init_node('voice_recognizer')  # initialize node
#     pub = rospy.Publisher('say', String, queue_size=10)  # create publisher

#     rate = rospy.Rate(10)  # 10hz
#     while not rospy.is_shutdown():
#         try:
#             speech_text = recognize_speech()  # recognize speech
#             rospy.loginfo(speech_text)  # log to console
#             pub.publish(speech_text)  # publish recognized speech text
#         except sr.UnknownValueError:
#             rospy.loginfo("could not understand audio")
#         except sr.RequestError as e:
#             rospy.loginfo("Could not request results from Google Speech Recognition service; {0}".format(e))
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         start_node()
#     except rospy.ROSInterruptException:
#         pass


import speech_recognition as sr

def print_voice1(mic_id=None):
    r = sr.Recognizer()
    try:
        with sr.Microphone() as source:
            print("Listening...")
            audio_data = r.record(source, duration=5)
            print("Recognizing...")
            text = r.recognize_google(audio_data, language="en-US")
            # text = r.recognize_whisper(audio_data, language="english")
            return text
    except AssertionError as ae:
        print("Error:", ae)
        return None
    except Exception as e:
        print("An unexpected error occurred:", e)
        return None
def print_voice(mic_id=id):
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_id) as source: #device_index=mic_id) as source:
        print("Listening...")
        audio_data = r.record(source, duration=5)  # record audio from the default microphone for 5 seconds
        print("Recognizing...")
        # using google speech recognition
        text = r.recognize_google(audio_data, language="en-US")

        # text = r.recognize_google(audio_data)
        return text


def print_voice2(mic_id=0):
    r = sr.Recognizer()

    try:
        with sr.Microphone() as source:
            print("Listening...")
            audio = r.listen(source)
            try:
                # user_input = r.recognize_whisper(audio, language="english") # Changed self.r to just r
                user_input = r.recognize_google(audio, language="en-US")
                print("Whisper thinks you said " + user_input)
                return user_input
            except SomeSpecificException: # Add the specific exception you're catching
                print("Error recognizing with whisper")
        
    except AssertionError as e:
        print("Could not request results. Error:", e)
    except Exception as e:
        # handle any other exceptions here
        print("An error occurred:", e)

# print(type(sr.Microphone.list_microphone_names())) #print all the microphones connected to your machine
## fetch the microphone with name steelseries in it 
# print(sr.Microphone.list_microphone_names())

mics=sr.Microphone.list_microphone_names()

print()

print()
print(mics)
for i in range(len(mics)):
    if "Razer" in mics[i]:
     
        print() 
        print("=="*50)
        print()
        print()
        print('Device name: ' ,mics[i])
        print('Device ID: ',i)
        mic_id = i
        break
print()    
print("=="*50)
print()
mic_id = 6

print(isinstance(mic_id, int))

# for index, name in enumerate(sr.Microphone.list_microphone_names()):
#     print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

try:
    print(print_voice2(mic_id=int(mic_id)))
except sr.UnknownValueError:
    print("Could not understand the audio. Please try again.")



# r = sr.Recognizer()
# with sr.Microphone(device_index=mic_id) as source:
#     print("Say something!")
#     audio = r.listen(source)

# try:
#     print("Whisper thinks you said " + r.recognize_whisper(audio, language="english"))
# except sr.UnknownValueError:
#     print("Whisper could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Whisper")