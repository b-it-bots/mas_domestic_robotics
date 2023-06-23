"""
Heartmet Challenge -2023
Authors: Melvin Paul Jacob, Zain Ul Haq
melvinpaul123@gmail.com, zainey4@gmail.com
"""
import rospy
import actionlib
from std_msgs.msg import String
import random
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_detect_gesture.msg import DetectGestureAction, DetectGestureGoal

class OrderTaking(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'order_taking',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations'],
                                   input_keys=['command'])

        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.objects_list = {"1":"Pringles","2":"Soup Can","3":"Windex Bottle","4":"T-shirt","5":"Spatula"}
        self.loc_list = {"1":"living_room","2":"hall","3":"reading_room","4":"dining"}
        # Define a list of possible friendly responses from the robot
        self.friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]
        self.gesture_try=0
        # Define a list of possible error responses from the robot
        self.error_responses = ["I'm sorry, I didn't understand that. Please try again.", "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding your gesture."]
        self.client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', DetectGestureAction)
        self.client.wait_for_server()
        self.goal = DetectGestureGoal()
        self.gesture_retries = 2
        self.pub_obj = rospy.Publisher('heartmet/target_object', String, queue_size=1)
            
    def gesture_call(self, state):
        self.goal.start = state
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        self.gesture_result = self.client.get_result()

    def list_items(self,item_dict):
        # self.say("I will be looking for object via hand gestures")
        # rospy.sleep(1)
        for locs in list(item_dict.items()):
            if locs[0]==0:
                self.say("For "+locs[1]+" show me "+ locs[0] + " finger")
            else:
                self.say("For "+locs[1]+" show me "+ locs[0] + " fingers")
            rospy.sleep(1)
        #self.say("fingers respectively")
        rospy.sleep(6)
        self.gesture_call(state = True)
        finger = self.gesture_result.gesture_selection
        if finger =="-1":
            item = None
        else:
            if finger in list(item_dict.keys()):
                item = set([item_dict[finger]])
            else:
                item = None
        return item
    
    def gesture_confirmation(self):
        self.say_this("Could you show thumbs up to confirm or thumbs down to decline", time_out=3)
        rospy.sleep(6)
        self.gesture_call(state = True)
        gest = self.gesture_result.gesture
        if gest=='Thumbs up':
            return True
        elif gest=='Thumbs down':
            return False
        else:
            return None
    
    # Define a function to generate a friendly response from the robot
    def generate_friendly_response(self):
        return random.choice(self.friendly_responses)

    # Define a function to generate an error response from the robot
    def generate_error_response(self):
        return random.choice(self.error_responses)

    def say_this(self, text, time_out=2):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        rospy.sleep(time_out)

    def comfirm_loop(self):
        confirmed = None
        self.say_this("I will be looking for confirmation via hand gestures", time_out=3)
        for tryi in range(self.gesture_retries):
            confirmed = self.gesture_confirmation()
            if confirmed==None:
                self.say_this(self.generate_error_response())
            else:
                return confirmed
        if confirmed==None:
            self.say_this("Sorry I could not confirm your response.")
        return confirmed
    
    def get_info(self, objects=None, locations=None):        
        if not objects or not locations:
            if not objects:
                for tryi in range(self.gesture_retries):
                    if tryi>0:
                        self.say_this(self.generate_error_response())
                    self.say_this("Could you tell me which object you'd like me to fetch", time_out=3)
                    #rospy.sleep(3)
                    objects = self.list_items(self.objects_list)
                    if objects:
                        break
                if not objects:
                    self.say_this("I'm sorry, but I didn't get any object to fetch. Feel free to call me if you want something.", time_out=3)
            if not locations and objects:
                for tryi in range(self.gesture_retries):
                    if tryi>0:
                        self.say_this(self.generate_error_response())
                    self.say_this(f"Could you tell me where you'd like me to fetch the {', '.join(objects)} from?", time_out=3)
                    #rospy.sleep(3)
                    locations = self.list_items(self.loc_list)
                    if locations:
                        break
                if not locations:
                    self.say_this("I'm sorry, but I didn't get any location to fetch from", time_out=3) #. I try to find the object from the environment.
                    # self.locations = set(["any_location"])
                    self.locations = None
        return objects, locations

    def get_props(self):
        prop=None
        for tryi in range(self.gesture_retries):
            if tryi>0:
                self.say_this("I'm sorry, but I could't properly catch which gesture you showed me. Can you please provide the gesture again?", time_out=4)
            #rospy.sleep(6)
            prop = self.list_items(self.props)
            if prop:
                break
        if not prop:
            prop = "both"
        return prop
    
    def execute(self, userdata):
        rospy.loginfo('Interacting through speech and gestures ...')
        '''self.say_this("Can I fetch somthing for you?")
        confirmed = self.comfirm_loop()
        if not confirmed:
            #self.say_this("If you change your mind, I will right here to serve you.", time_out=3)
            #self.say_this("I will fetch now", time_out=2)
            #userdata.destination_locations = ['dining'] #['dining_table']
            #self.pub_obj.publish('pringles')
            return 'failed'
        else:'''
        self.say_this("Sorry I could not hear you properly. You can interact with me via gestures", time_out=4)
        self.say_this("What can I fetch for you?")
        #self.say_this("Could you tell me object and also its location if you can", time_out=3)
        objects, locations = None, None
        for i in range(self.gesture_retries):
            if i>0:
                self.say_this("I'm sorry, but I could't properly catch the object and or the location that you might have said. I will be looking for gestures")
            objects, locations = self.get_info(objects, locations)
            if objects and locations:
                self.say_this(f"Just to confirm, You want me to bring {', '.join(objects)} from {', '.join(locations)}. Is that correct?", time_out=3)
                confirmed = self.comfirm_loop()
                if not confirmed:
                    self.say_this("Can you tell me which is wrong, object, location or both", time_out=3)
                    prop = self.get_props()
                    if prop=="object":
                        objects = None
                    elif prop=="location":
                        locations = None
                    else:
                        objects, locations = None, None
                else:
                    loc = list(locations)[0]
                    obj = list(objects)[0]
                    userdata.destination_locations = [loc] #['dining_table']
                    self.pub_obj.publish(obj)
                    # userdata.objects = [obj]
                    self.say_this(f"I will fetch {', '.join(objects)} from {', '.join(locations)}.", time_out=3)
                    return 'succeeded'
        #self.say_this("Sorry I could not identify the object and or location. Please try again later",time_out=3)
        #self.say_this("I will fetch now", time_out=2)
        #userdata.destination_locations = ['dining'] #['dining_table']
        #self.pub_obj.publish('pringles')
        return 'failed'