import cv2
import mediapipe as mp
import numpy as np
import time
from collections import deque 
#from sklearn.preprocessing import MinMaxScaler
from tensorflow.keras.models import load_model
import tensorflow as tf
import os

class EuclideanDistTracker:
    def __init__(self, min_dist=30, max_disp=50):
        # Store the center positions of the objects
        self.min_dist = min_dist
        self.max_disp = max_disp
        self.center_points = {}
        self.disappeared = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0
            
    def centroid(self, rect):
        x1, y1, x2, y2 = rect
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        return np.array([cx, cy])

    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []
        # Get center point of new object
        for rect in objects_rect:
            area = (rect[1]-rect[3])*(rect[0]-rect[2])
            center = self.centroid(rect)
            # Find out if that object was detected already
            same_object_detected = False
            obs = np.array(list(self.center_points.values()))
            ids = np.array(list(self.center_points.keys()))
            if len(obs)>0:
                dists = np.linalg.norm(center-obs, axis=1)
                closest_indx = np.argmin(dists)
                if dists[closest_indx] < self.min_dist:
                    id = ids[closest_indx]
                    self.center_points[id] = center
                    #print(self.center_points)
                    objects_bbs_ids.append([rect, id,area])
                    same_object_detected = True
                    self.disappeared[id] = 0

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = center
                objects_bbs_ids.append([rect, self.id_count,area])
                self.disappeared[self.id_count] = 0
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            re, object_id, ar = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center
        
        disap = [x for x in list(self.center_points.keys()) if x not in list(new_center_points.keys())]
        for i in disap:
            self.disappeared[i] += 1
            if self.disappeared[i]<self.max_disp:
                new_center_points[i] = self.center_points[i]
        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

class PoseDetector():
    def __init__(self, hand_model="", head_model="", seq_len=10):
        if not hand_model=="":
            hand_classifiername, hand_classifier_extension = os.path.splitext(hand_model)
            if hand_classifier_extension==".tflite":
                self.hand_interpreter = tf.lite.Interpreter(model_path=hand_model,num_threads=1)
                self.hand_interpreter.allocate_tensors()
                self.hand_input_details = self.hand_interpreter.get_input_details()
                self.hand_output_details = self.hand_interpreter.get_output_details()
                self.hand_lite=True
            else:
                self.model_hand = load_model(hand_model)
                self.hand_lite=False
                
        if not head_model=="":
            head_classifiername, head_classifier_extension = os.path.splitext(hand_model)
            if head_classifier_extension==".tflite":
                self.head_interpreter = tf.lite.Interpreter(model_path=head_model,num_threads=1)
                self.head_interpreter.allocate_tensors()
                self.head_input_details = self.head_interpreter.get_input_details()
                self.head_output_details = self.head_interpreter.get_output_details()
                self.head_lite=True
            else:
                self.model_head = load_model(head_model)
                self.head_lite=False
        
        self.classify_part= "head"
        self.seq_len = seq_len
        self.hand_gestures = {0: "Stop sign", 1: "Thumbs down", 2: "Waving", 3: "Pointing",
                         4: "Calling someone", 5: "Thumbs up", 6: "Wave someone away", 7: "Others"}
        self.head_gestures = {0: "Nodding",  1: "Shaking head", 2: "Others"}
        self.pTime = 0
        self.d = deque(maxlen=60)
        self.fps=0
        self.keys = deque(maxlen=self.seq_len)
        self.tipIds = [4, 8, 12, 16, 20]
        self.img_w = 640
        self.img_h =  480
        self.prev_x = 0
        self.prev_y = 0
        self.focal_length = 1 * self.img_w
        self.dist_matrix = np.zeros((4, 1), dtype=np.float64)
        self.cam_matrix = np.array([[self.focal_length, 0, self.img_h / 2],
                                    [0, self.focal_length, self.img_w / 2],
                                    [0, 0, 1]])
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_face = mp.solutions.face_mesh
        self.face =  self.mp_face.FaceMesh(max_num_faces=1,min_detection_confidence=0.5,min_tracking_confidence=0.5,refine_landmarks=False)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity=0,min_detection_confidence=0.5,min_tracking_confidence=0.5)
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

    def get_head_rot(self,result1):
        if result1.multi_face_landmarks:
            for face_landmarks in result1.multi_face_landmarks:
                face_3d = []
                face_2d = []
                for idx, lm in enumerate(face_landmarks.landmark):
                    if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                        if idx == 1:
                            center = lm.y * self.img_h
                        x, y = int(lm.x * self.img_w), int(lm.y * self.img_h)
                        face_2d.append([x, y])
                        face_3d.append([x, y, lm.z])       
                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)
                success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, self.cam_matrix, self.dist_matrix)
                rmat, jac = cv2.Rodrigues(rot_vec)
                angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
                x = angles[0] * 360
                y = angles[1] * 360
                dx, dy = self.prev_x-x, self.prev_y-y
                self.prev_x = x
                self.prev_y = y
                return np.array([dx, dy]), center
        else:
            return np.array([0,0]), self.img_h

    def fingersUp(self,results2):
        fingers = []
        if results2.multi_hand_landmarks:
            for hand_landmarks in results2.multi_hand_landmarks:
                lmList = np.array([[res.x, res.y] for res in hand_landmarks.landmark])
                # Thumb
                if lmList[self.tipIds[1]-3][0] > lmList[self.tipIds[4]-3][0]:
                    if lmList[self.tipIds[0]][0] >= lmList[self.tipIds[0] - 1][0]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
                else:
                    if lmList[self.tipIds[0]][0] <= lmList[self.tipIds[0] - 1][0]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
                # Fingers
                for id in range(1, 5):
                    if lmList[self.tipIds[id]][1] < lmList[self.tipIds[id] - 2][1]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
            totalFingers = fingers.count(1)
        else:
            totalFingers=-1
        return totalFingers

    def update_keys(self, results1, results2):
        face_rot, lh, rh, ys = self.data2array(results1, results2)
        key = np.concatenate([face_rot.flatten(),lh.flatten(),rh.flatten()])
        self.keys.append(key)
    
    def classify(self):
        if np.any(self.keys):
            X = np.expand_dims(self.keys, axis=0)
            X3 = X[:,:,0:2]
            if np.any(X3):
                if self.head_lite:
                    X3 = np.array(X3, dtype=np.float32)
                    head_input_details_tensor_index = self.head_input_details[0]['index']
                    self.head_interpreter.set_tensor(head_input_details_tensor_index,X3)
                    self.head_interpreter.invoke()
                    head_output_details_tensor_index = self.head_output_details[0]['index']
                    head_pred1 = self.head_interpreter.get_tensor(head_output_details_tensor_index)
                else:
                    head_pred1 = self.model_head.predict(X3,verbose=0)
                head_prob = np.max(head_pred1)
                head_pred = np.argmax(head_pred1,axis=1)
                head_gest = self.head_gestures[head_pred[0]]
            else:
                head_gest, head_prob = "Others", 1.0
            
            #hands
            X = X.reshape((X.shape[0], self.seq_len, int(X.shape[2]/2),2))
            X1 = X[:,:,1:22,:]
            X2 = X[:,:,22:,:]
            if np.any(X1) or np.any(X2):
                if self.hand_lite:
                    X1 = np.array(X1, dtype=np.float32)
                    X2 = np.array(X2, dtype=np.float32)
                    hand_input_details_tensor_index = self.hand_input_details[0]['index']
                    self.hand_interpreter.set_tensor(hand_input_details_tensor_index,X1)
                    hand_input_details_tensor_index1 = self.hand_input_details[1]['index']
                    self.hand_interpreter.set_tensor(hand_input_details_tensor_index1,X2)
                    self.hand_interpreter.invoke()
                    hand_output_details_tensor_index = self.hand_output_details[0]['index']
                    hand_pred1 = self.hand_interpreter.get_tensor(hand_output_details_tensor_index)
                else:
                    hand_pred1 = self.model_hand.predict([X1,X2],verbose=0)
                hand_prob = np.max(hand_pred1)
                hand_pred = np.argmax(hand_pred1,axis=1)
                hand_gest = self.hand_gestures[hand_pred[0]]
            else:
                hand_gest, hand_prob = "Others", 1.0
            
            #print(self.head_gestures[head_pred[0]], head_prob, self.hand_gestures[hand_pred[0]], hand_prob)
            self.keys = deque(maxlen=self.seq_len)
            if head_gest=="Others" and hand_gest=="Others":
                return head_gest, head_prob
            elif head_gest!="Others" and hand_gest=="Others":
                return head_gest, head_prob
            elif head_gest=="Others" and hand_gest!="Others":
                return hand_gest, hand_prob
            else:
                if head_prob>hand_prob:
                    return head_gest, head_prob
                else:
                    return hand_gest, hand_prob
        else:
            return "None", 1.0
    
    def mediapipe_detection(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results1 = self.face.process(image)
        results2 = self.hands.process(image)
        return results1, results2
    
    def draw(self, image, pose):
        #pose1 = pose[0:15]#np.delete(pose, (0,21), axis = 0)
        for i in pose:
            cv2.circle(image, (int(i[0]),int(i[1])), 2, (255,255,0), -1)
        return image    
        
    def draw_landmarks(self, image, results1, results2):
        # Draw face connections
        if results1.multi_face_landmarks:
          for face_landmarks in results1.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(image=image,landmark_list=face_landmarks,
                                               connections=self.mp_face.FACEMESH_CONTOURS,landmark_drawing_spec=None,
                                               connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style())
        # Draw hand connections
        if results2.multi_hand_landmarks:
            for hand_landmarks in results2.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(image,hand_landmarks,self.mp_hands.HAND_CONNECTIONS,
                                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                        self.mp_drawing_styles.get_default_hand_connections_style())
        return image
        
    def get_fps(self):
        self.cTime = time.time()
        self.fps = 1 / (self.cTime - self.pTime)
        self.d.append(self.fps)
        self.fps = sum(self.d)/len(self.d)
        self.pTime = self.cTime
        return self.fps
    
    def normalize(self,key):
        key = key-key[0]
        max_value = np.amax(np.abs(key))
        key = key/max_value
        key = np.array(key, dtype=np.float32)
        #scaler = MinMaxScaler()
        #face = scaler.fit_transform(face)
        return key
    
    def data2array(self, results1, results2):
        face_rot, head_y = self.get_head_rot(results1)
        lh= np.zeros((21, 2))
        rh= np.zeros((21, 2))
        lh_y = self.img_h
        rh_y = self.img_h
        if results2.multi_hand_landmarks:
          for hand_landmarks, handedness in zip(results2.multi_hand_landmarks,results2.multi_handedness):
              side = str(handedness.classification[0].label[0:]).lower()
              hand = np.array([[res.x, res.y] for res in hand_landmarks.landmark])#.flatten()
              h_y = hand[9][1]*self.img_h
              hand = self.normalize(hand)
              if side=='left':
                  lh = hand
                  lh_y = h_y
              else:
                  rh = hand
                  rh_y = h_y
        return face_rot, lh, rh, (head_y,lh_y,rh_y)
    
    def face_detect(self, image, draw=True):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.face_detection.process(image)
        h, w = image.shape[:2]
        face_boxes = []
        if results.detections:
          for detection in results.detections:
              bboxC = detection.location_data.relative_bounding_box
              xmin, ymin, hw, hh = int(bboxC.xmin * w), int(bboxC.ymin * h), int(bboxC.width * w), int(bboxC.height * h)
              xmax, ymax = xmin+hw, ymin+hh
              face_boxes.append([xmin, ymin, xmax, ymax])
        return face_boxes

class MultiPerDetector():
    def __init__(self,rp, num_person, hand_model="", head_model="", seq_len=10):
        #dir_path = os.path.dirname(os.path.realpath(__file__))
        #print(dir_path+"/"+per_path)
        self.seq_len = seq_len
        self.num_person = num_person
        self.tracker = EuclideanDistTracker()
        for i in range(self.num_person):
            globals()["per"+str(i)] = PoseDetector(hand_model=hand_model, head_model=head_model, seq_len=seq_len)
        self.rp = rp
        self.avg_len = 10
        self.results = deque(maxlen=self.avg_len)
        
    def det_per(self,frame):
        bboxs = self.rp.detect_person(frame)
        objects_bbs_ids = self.tracker.update(bboxs)
        #print(bboxs,objects_bbs_ids)
        return objects_bbs_ids
    
    def multi_per_gesture(self,image,sort="id",viz=False):
        gestures = []
        objects_bbs_ids = self.det_per(image)
        #print(objects_bbs_ids)
        if sort =="id":
            sorted_objects_bbs_ids = sorted(objects_bbs_ids,key=lambda x: x[1])
        elif sort=="area":
            sorted_objects_bbs_ids = sorted(objects_bbs_ids,key=lambda x: x[2])[::-1]
        for i, bbox in enumerate(sorted_objects_bbs_ids):
            if i<self.num_person:
                person = image[bbox[0][1]:bbox[0][3],bbox[0][0]:bbox[0][2]]
                results1, results2 = globals()["per"+str(i)].mediapipe_detection(person)
                globals()["per"+str(i)].update_keys(results1, results2)
                if len(globals()["per"+str(i)].keys)==self.seq_len: #i%10==0 and 
                    gest, prob = globals()["per"+str(i)].classify()
                    finger_count = globals()["per"+str(i)].fingersUp(results2)
                    gestures.append([gest, prob, bbox[1], bbox[0], str(finger_count)]) #gest, prob, id, bbox, finger_count
                    globals()["per"+str(i)].keys = deque(maxlen=self.seq_len)
                if viz:
                    per = globals()["per"+str(i)].draw_landmarks(person, results1, results2)
                    image[bbox[0][1]:bbox[0][3],bbox[0][0]:bbox[0][2]] = per
            if viz:
                cv2.rectangle(image, (bbox[0][0], bbox[0][1]), (bbox[0][2], bbox[0][3]), (255,255,255), thickness=2)
                cv2.putText(image, str(bbox[1]), (bbox[0][0], int(bbox[0][1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        if gestures!=[]:
            self.results.append(gestures)
        return image, gestures
    
    def get_all_indices(self,lst, element):
        return [i for i in range(len(lst)) if lst[i] == element]
    
    def get_ind_vals(self,lst, indices):
        return [lst[i] for i in indices]
    
    def avg_per_gesture(self):
        avg_gestures = []
        if len(self.results)==self.avg_len:
            for per in range(self.num_person):
                fingers = []
                gestures= []
                probs=[]
                ids = []
                bboxs= []
                for i in range(len(self.results)):
                    if self.results[i]!=[]:
                        fingers.append(self.results[i][per][4])
                        gestures.append(self.results[i][per][0])
                        probs.append(self.results[i][per][1])
                        ids.append(self.results[i][per][2])
                        bboxs.append(self.results[i][per][3])
                if fingers != []:
                    finger = max(set(fingers), key=fingers.count)
                    gesture = max(set(gestures), key=gestures.count)
                    #print(finger,gesture)
                    #print(self.get_all_indices(gestures,gesture))
                    prob = np.average(self.get_ind_vals(probs,self.get_all_indices(gestures,gesture)))
                    idv = max(set(self.get_ind_vals(ids,self.get_all_indices(gestures,gesture))), key=self.get_ind_vals(ids,self.get_all_indices(gestures,gesture)).count)
                    bbox = bboxs[gestures.index(gesture)]
                    avg_gestures.append([gesture, prob, idv, bbox, finger])
            self.results = deque(maxlen=self.avg_len)
            return avg_gestures
        else:
            return []
        
    
    def multi_per_poses(self,image,sort="id",viz=False):
        poses = []
        objects_bbs_ids = self.det_per(image)
        if sort =="id":
            sorted_objects_bbs_ids = sorted(objects_bbs_ids,key=lambda x: x[1])
        elif sort=="area":
            sorted_objects_bbs_ids = sorted(objects_bbs_ids,key=lambda x: x[2])
        for i, bbox in enumerate(sorted_objects_bbs_ids):
            if i<self.num_person:
                person = image[bbox[0][1]:bbox[0][3],bbox[0][0]:bbox[0][2]]
                results1, results2 = globals()["per"+str(i)].mediapipe_detection(person)
                poses.append(results1, results2, bbox[1], bbox[0])
            if viz:
                cv2.rectangle(image, (bbox[0][0], bbox[0][1]), (bbox[0][2], bbox[0][3]), (255,255,255), thickness=2)
                cv2.putText(image, str(bbox[1]), (bbox[0][0], int(bbox[0][1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                per = globals()["per"+str(i)].draw_landmarks(person, results1, results2)
                image[bbox[0][1]:bbox[0][3],bbox[0][0]:bbox[0][2]] = per
        return image, poses