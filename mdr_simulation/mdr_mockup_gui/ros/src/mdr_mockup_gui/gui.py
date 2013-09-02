#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool

from mcr_perception_msgs.msg import ObjectList
from mcr_perception_msgs.msg import Object

from Tkinter import *

from mcr_speech_msgs.msg import RecognizedSpeech


class Application(Frame):


	def __init__(self, master=None):

		Frame.__init__(self, master)

		#init the ROS node
		rospy.init_node('mdr_mockup_gui')

		self.pub = rospy.Publisher('~recognized_speech', RecognizedSpeech, latch=True)
		self.set_objects_publisher = rospy.Publisher('~set_recognized_objects', ObjectList)
		self.set_doorstate_publisher = rospy.Publisher('~change_door_status', Bool)

		#Class variables
		self.radio_button_selection = StringVar() # stores the selection
		self.checkbutton_selection_list = []
		
		# button array for every category (separate arrays for for alignment)
		self.radiobuttons_speech_commands = []
		self.radiobuttons_speech_objects = []
		self.radiobuttons_speech_names = []
		self.radiobuttons_speech_locations = []
		self.checkbuttons_objects_identified = []

		self.openDoorStatus = False

		self.grid()
		self.createWidgets()
				
		
	#inserts radiobuttons
	def insertSpeechRadioButtons(self, speech_data, radioButtonArray, column, headline):
		
		separator = Frame(relief=SUNKEN, border = 2)
		separator.grid(column = column, row = 0, sticky=N)
		
		label = Label(separator, text=headline, anchor = W)
		label.grid()
		
		#create buttons
		for mystring in speech_data:			
			radioButtonArray.append(Radiobutton(separator, text=mystring, variable=self.radio_button_selection, 
			value=mystring, command = self.sendSpeech, anchor=W))	
		
		#place buttons in column
		for btn in radioButtonArray:
			btn.grid(column = 0, sticky=W)
		
		
		#inserts radiobuttons
	def insertObjectRecognition(self, checkButtonArray, column, headline):
		
		separator = Frame(relief=SUNKEN, border = 2)
		separator.grid(column = column, row = 0, sticky=N)
		
		label = Label(separator, text=headline)
		label.grid()
		
		data = rospy.get_param("~speech_objects", ["<~speech_objects empty>"])

		#create buttons
		for mystring in data:			
			data = IntVar()			
			self.checkbutton_selection_list.append(data)
			
			checkButtonArray.append(Checkbutton(separator, text=mystring, variable=data, 
			command = self.setObjects, anchor=W))	
		
		#place buttons in column
		for btn in checkButtonArray:
			btn.grid(column = 0, sticky=W)
		

	def createWidgets(self):
		
		#freetext speech textfield
		self.speech_recognition_input = Entry(master=self)
		self.speech_recognition_input.grid()			
		
		self.send_speech_button = Button ( self, text='Send Text Speech', command=self.sendSpeechFree )
		self.send_speech_button.grid(sticky=N)
	
		self.open_door_button = Button ( self, text='Switch Door State', command=self.openDoor )
		self.open_door_button.grid(sticky=N)
	
		locations = rospy.get_param("~speech_places", ["<~speech_places empty>"])
		#locations = locations + rospy.get_param("/script_server/brsu_speech_grasp_locations")
	
		self.insertSpeechRadioButtons(rospy.get_param("~speech_commands",["<~speech_commands empty>"]), self.radiobuttons_speech_commands, 1, "commands")
		self.insertSpeechRadioButtons(rospy.get_param("~speech_objects", ["<~speech_objects empty>"]), self.radiobuttons_speech_objects, 2, "objects")
		self.insertSpeechRadioButtons(rospy.get_param("~speech_names", ["<~speech_names empty>"]), self.radiobuttons_speech_names, 3, "names")
		self.insertSpeechRadioButtons(locations, self.radiobuttons_speech_locations, 3, "locations")
		self.insertObjectRecognition(self.checkbuttons_objects_identified, 4, "identified objects")
			
	def sendSpeech(self):
		words = self.radio_button_selection.get().split(" ")
		confidences = []
		
		for word in words:
			confidences.append(0.9)
		data = RecognizedSpeech(self.radio_button_selection.get(), self.radio_button_selection.get(), 0.9, words, confidences)
		self.pub.publish(data)
	
	def sendSpeechFree(self):
		words = self.speech_recognition_input.get().split(" ")
		confidences = []
		
		for word in words:
			confidences.append(0.9)
		data = RecognizedSpeech(self.speech_recognition_input.get(), self.speech_recognition_input.get(), 0.9, words, confidences)
		self.pub.publish(data)

	def openDoor(self):
		self.openDoorStatus = not self.openDoorStatus
		status = Bool()
		status.data = self.openDoorStatus

		self.set_doorstate_publisher.publish(status)


	def setObjects(self):

		recognized_objects = ObjectList()

		#test all checkbuttons if active
		for i in range(len(self.checkbutton_selection_list)):
			object_names = rospy.get_param("~identified_objects", ["<~identified_objects empty>"])		
			if self.checkbutton_selection_list[i].get() > 0: 			
				found_object = Object()
				found_object.name = object_names[i]
				found_object.pose.pose.position.x = 0.5
				found_object.pose.pose.position.y = 0.3
				found_object.pose.pose.position.z = 0.5
				recognized_objects.objects.append(found_object)
		self.set_objects_publisher.publish(recognized_objects)		
		

def main():
	app = Application()
	app.master.title("MDR Mockup GUI")
	app.mainloop()

