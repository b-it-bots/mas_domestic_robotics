import random
import speech_recognition as sr
from transformers import pipeline
import spacy
from spacy.tokens import DocBin
from tqdm import tqdm

nlp_ner = spacy.load("my-model") 

# Define a list of possible friendly responses from the robot
friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]

# Define a list of possible error responses from the robot
error_responses = ["I'm sorry, I didn't understand that. Please try again.", "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding you. Please speak more clearly."]

# Initialize the speech recognition module
r = sr.Recognizer()

# Initialize the named entity recognition pipeline from transformers
# nlp = pipeline("ner", model="dslim/bert-base-NER", tokenizer="dslim/bert-base-NER")

# Define a function to generate a friendly response from the robot
def generate_friendly_response():
    return random.choice(friendly_responses)

# Define a function to generate an error response from the robot
def generate_error_response():
    return random.choice(error_responses)

# Define a function to process user input and generate a response
def process_user_input(user_input):
    # Use the named entity recognition pipeline to extract object and location entities from the user input
    entities = nlp_ner(user_input)
    obj_lst = []
    loc_lst = []
    
    # Loop through the extracted entities and store the objects and locations in separate lists
    objects=[]
    locations=[]
    for ent in entities.ents:
    #     print(type(ent.text),type(ent.label_))

        print(ent.text, ent.label_)
        if ent.label_=='OBJ':
            objects.append(ent.text)
        elif ent.label_=='LOC':
            locations.append(ent.text)
            
    return set(objects), set(locations)

# Define a function to listen for voice input and pass it to the chatbot
def listen():
    with sr.Microphone() as source:
        print("Say something!")
        audio = r.listen(source)
    
    try:
        user_input = r.recognize_google(audio)
#         user_input = "please turn on the light in the living room"
        print("You said:", user_input)
        objects, locations = process_user_input(user_input)
        print(objects, locations)
    except sr.UnknownValueError:
        print(generate_error_response())
    except sr.RequestError as e:
        print("Sorry, there was an error processing your request. Please try again later.")

# Define a loop to keep the chatbot running and listening for voice input
while True:
    listen()