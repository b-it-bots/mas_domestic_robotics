import mediapipe as mp
import cv2
#import time
#from mediapipe.python._framework_bindings import timestamp

class Person_Detector():
    def __init__(self):
        #Timestamp = timestamp.Timestamp
        model_path_dir = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_perception_actions/mdr_detect_gesture/model/"
        model_path = model_path_dir+'mobilenetv2_ssd_256_uint8.tflite'
        BaseOptions = mp.tasks.BaseOptions
        ObjectDetector = mp.tasks.vision.ObjectDetector
        ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
        VisionRunningMode = mp.tasks.vision.RunningMode
        self.frame_index = 1
        self.fps = 30
        self.results= None

        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            max_results=5,
            result_callback=self.print_result)
        
        self.detector = ObjectDetector.create_from_options(options)
        
    def print_result(self, result:mp.tasks.components.containers.DetectionResult, output_image: mp.Image, timestamp_ms: int):
        #print('detection result: {}'.format(result))
        self.results = result
        
    def detect_person(self,frame):
        bboxs = []
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        # Calculate the timestamp of the current frame
        frame_timestamp_ms = int(1000 *  self.frame_index/self.fps)
        #print(ts.seconds())
        # Perform object detection on the video frame.
        self.detector.detect_async(mp_image,frame_timestamp_ms)
        if self.results!=None:
            #print('detection result1: {}'.format(self.results))
            det_res = self.results.detections
            for res in det_res:
                #print(res)
                if res.categories[0].category_name == 'person' and res.categories[0].score > 0.5:
                    bbox = res.bounding_box
                    x1,y1,w,h = bbox.origin_x, bbox.origin_y, bbox.width, bbox.height
                    x2,y2 = x1+w, y1+h
                    bboxs.append([x1,y1,x2,y2])
                    #cv2.rectangle(frame, (x1, y1), (x2, y2), (255,255,255), thickness=2)
                    #cv2.putText(image, str(bbox[1]), (bbox[0][0], int(bbox[0][1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                    #print("------------")
        self.frame_index=self.frame_index+1
        return bboxs