from mas_perception_libs import YoloImageDetector
from geometry_msgs.msg import PoseStamped
#from mdr_detect_gesture.inference import PoseDetector

class FallDetector(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'FallDetector',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations'],
                                   input_keys=['command'])
        self.obj_detector = YoloImageDetector()
        rospy.Subscriber('/heartmet/target_object', String, self.object_callback)
        self.object_data = 'pringles'
        self.thresh_fall = 0.2

    def object_callback(self, data):
        self.object_data = data
    
    def detect_object(self, image, objectoi:str):
        detected_obj_data, bbox = self.obj_detector.single_img_detect(image, objectoi) #or self.object_data.data
        if detected_obj_data != []:
            print("object of interest in frame")
            #find 3d location.
            obj_loc = PoseStamped()
            if obj_loc.pose.position.z < self.thresh:
                print("object on the ground")
        else:
            print("object not found it maybe on the ground")