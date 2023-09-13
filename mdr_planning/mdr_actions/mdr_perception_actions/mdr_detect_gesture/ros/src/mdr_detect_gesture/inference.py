import cv2
import mediapipe as mp
import numpy as np
import time
from collections import deque 
from tensorflow.keras.models import load_model
import math

class EuclideanDistTracker:
    def __init__(self, max_id, max_disp=50):
        # Store the center positions of the objects
        self.max_id = max_id
        self.max_disp = max_disp
        self.center_points = {}
        self.disappeared = {}
        for i in range(self.max_id):
            self.center_points[i] = np.array([10000, 10000])
            self.disappeared[i] = self.max_disp
            
    def centroid(self, rect):
        x1, y1, x2, y2 = rect
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        return np.array([cx, cy])
    
    def calculate_distance(self,point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def assign(self, objects_rect):
        objects_bbs_ids={}
        for q in range(self.max_id):
            objects_bbs_ids[q] = []
        areas = []
        for rect in objects_rect:
            center = self.centroid(rect)
            area = (rect[1]-rect[3])*(rect[0]-rect[2])
            areas.append([rect,area,center])
        sorted_areas = []
        if len(areas)>0:
            sorted_areas = sorted(areas, key=lambda x: x[1], reverse=True)[:self.max_id]
            #print(sorted_areas)
            dists={}
            #print(len(sorted_areas))
            #print(self.center_points)
            for idn in self.center_points:
                for q in range(len(sorted_areas)):
                    dists[(idn,q)] = self.calculate_distance(sorted_areas[q][2],self.center_points[idn])
            #print(dists)
            dist_items = list(dists.items())
            for i in range(len(sorted_areas)):
                dist_items.sort(key=lambda x: x[1])
                idn = dist_items[0][0][0]
                idx = dist_items[0][0][1]
                self.center_points[idn] = sorted_areas[idx][2]
                objects_bbs_ids[idn]=sorted_areas[idx][0]
                self.disappeared[idn] = 0
                remaining = []
                for x in range(len(dist_items)):
                    if dist_items[x][0][0]!=idn and dist_items[x][0][1]!=idx:
                        remaining.append(dist_items[x])
                dist_items = remaining

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for idn in objects_bbs_ids:
            if objects_bbs_ids[idn]!=[]:
                center = self.center_points[idn]
                new_center_points[idn] = center
        
        disap = [x for x in list(self.center_points.keys()) if x not in list(new_center_points.keys())]
        for i in disap:
            self.disappeared[i] += 1
            if self.disappeared[i]>=self.max_disp:
                self.center_points[i] = np.array([10000, 10000])
        return objects_bbs_ids

class PoseDetector():
    def __init__(self, hand_model="", head_model="", fing_model="", seq_len=10, avg_len=6, classify_thresh=0.45):
        if not hand_model=="":
            self.model_hand = load_model(hand_model)
                
        if not head_model=="":
            self.model_head = load_model(head_model)
            
        if not fing_model=="":
            self.model_fing = load_model(fing_model)
        self.classify_thresh = classify_thresh
        self.seq_len = seq_len
        self.hand_gestures = {0: "Stop sign", 1: "Thumbs down", 2: "Waving", 3: "Pointing",
                         4: "Calling someone", 5: "Thumbs up", 6: "Wave someone away", 7: "Others"}
        self.head_gestures = {0: "Nodding",  1: "Shaking head", 2: "Others"}
        self.num_gests = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "Others"}
        self.pTime = 0
        self.d = deque(maxlen=60)
        self.fps=0
        self.tipIds = [4, 8, 12, 16, 20]
        self.img_w = 640
        self.img_h =  480
        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0
        self.focal_length = 1 * self.img_w
        self.dist_matrix = np.zeros((4, 1), dtype=np.float64)
        self.cam_matrix = np.array([[self.focal_length, 0, self.img_h / 2],
                                    [0, self.focal_length, self.img_w / 2],
                                    [0, 0, 1]])
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_face = mp.solutions.face_mesh
        self.face =  self.mp_face.FaceMesh(max_num_faces=1,min_detection_confidence=0.5,min_tracking_confidence=0.5,refine_landmarks=False,static_image_mode=True)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2,model_complexity=0,min_detection_confidence=0.5,min_tracking_confidence=0.5,static_image_mode=True)

    def get_head_rot(self,result1):
        if result1.multi_face_landmarks:
            #for face_landmarks in result1.multi_face_landmarks:
            face_3d = []
            face_2d = []
            lms_rot = [33,263,1,61,291,199]
            for idx in lms_rot:
                #for idx, lm in enumerate(face_landmarks.landmark):
                lm = result1.multi_face_landmarks[0].landmark[idx]
                #if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
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
            z = angles[2] * 360
            dx, dy, dz = self.prev_x-x, self.prev_y-y, self.prev_z-z
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            #print(np.array([dx, dy]))
            return np.array([dx, dy]) #np.array([dx, dy, dz])
        else:
            return np.array([0,0]) #np.array([0,0,0])
    
    def calculate_angles(self, point):
        # Calculate the vector representing the line
        vector = point[:, 1] - point[:, 0]
        # Calculate the angles along the x-axis, y-axis, and z-axis
        angle_z = []
        for i in vector:
            #angle_x = np.arctan2(vector[1], vector[0])
            #angle_y = np.arctan2(vector[0], vector[2])
            #angle_z = np.arctan2(vector[1], vector[2])
            angle_z.append(np.arctan2(i[1], i[2]))
        # Convert the angles from radians to degrees
        angle_z = np.degrees(angle_z)
        condition = np.logical_and(angle_z > 70, angle_z < 120)
        # Apply the mask to filter the angles
        filtered_angles = angle_z[condition]
        values, counts = np.unique(filtered_angles, return_counts=True)
        ind = np.argmax(counts)
        value = values[ind]
        return value
    
    def combine_results(self,res1,res1_prob,res2,res2_prob):
        if res1=="Others" and res2=="Others":
            return res1, res1_prob
        elif res1!="Others" and res2=="Others":
            return res1, res1_prob
        elif res1=="Others" and res2!="Others":
            return res2, res2_prob
        else:
            if res1_prob>res2_prob:
                return res1, res1_prob
            else:
                return res2, res2_prob

    def fingersUp(self,keys):
        keys = np.array(keys)
        if np.any(keys[:,2:]):
            lhs = np.array(keys[:,2:44])
            rhs = np.array(keys[:,44:])
            
            #lh
            if np.any(lhs):
                lh_pred1 = self.model_fing.predict(lhs,verbose=0)
                value, lh_prob = self.process_res(lh_pred1,len(self.num_gests)-1)
                lh_gest = self.num_gests[value]
            else:
                lh_gest, lh_prob = "Others", 1.0
                
            #rh
            if np.any(rhs):
                rh_pred1 = self.model_fing.predict(rhs,verbose=0)
                value, rh_prob = self.process_res(rh_pred1,len(self.num_gests)-1)
                rh_gest = self.num_gests[value]
            else:
                rh_gest, rh_prob = "Others", 1.0
            return self.combine_results(lh_gest, lh_prob, rh_gest, rh_prob)
        else:
            return "Others", 1.0
    
    def rolling_window2D(self,a,n,step=3):
        return a[np.arange(a.shape[0]-n+1)[:,None] + np.arange(n)][::step, :]
    
    def process_res(self, arr, default_value):
        max_indices = np.argmax(arr, axis=1)
        #print(max_indices)
        mask = np.max(arr, axis=1) < self.classify_thresh
        max_indices[mask] = default_value
        #print(max_indices)
        values, counts = np.unique(max_indices, return_counts=True)
        ind = np.argmax(counts)
        value = values[ind]
        arr_probs = np.mean(np.array(arr), axis=0) 
        return value, arr_probs[value]
    
    def classify(self, keys, rots):
        rots = np.array(rots)
        if np.any(keys):
            keys = np.array(keys)
            X = self.rolling_window2D(keys,self.seq_len)
            #X = np.expand_dims(keys, axis=0)
            X3 = X[:,:,0:2]
            X3 = np.absolute(X3)
            if np.any(X3):
                head_pred1 = self.model_head.predict(X3,verbose=0)
                value, head_prob = self.process_res(head_pred1,len(self.head_gestures)-1)
                head_gest = self.head_gestures[value]
            else:
                head_gest, head_prob = "Others", 1.0

            #hands
            X = X.reshape((X.shape[0], self.seq_len, int(X.shape[2]/2),2))
            X1 = X[:,:,1:22,:]
            X2 = X[:,:,22:,:]
            if np.any(X1) or np.any(X2):
                hand_pred1 = self.model_hand.predict([X1,X2],verbose=0)
                value, hand_prob = self.process_res(hand_pred1,len(self.hand_gestures)-1)
                hand_gest = self.hand_gestures[value]
                if hand_gest not in ["Pointing","Thumbs up","Thumbs down", "Others"]:
                    if np.any(rots[:,0:2,:]):
                        zx1 = self.calculate_angles(rots[:,0:2,:])
                    else:
                        zx1=0
                    if np.any(rots[:,2:,:]):
                        zx2 = self.calculate_angles(rots[:,2:,:])
                    else:
                        zx2=0
                    if not (zx1 or zx2):
                        hand_gest, hand_prob = "Others", 1.0
            else:
                hand_gest, hand_prob = "Others", 1.0
            return self.combine_results(head_gest, head_prob, hand_gest, hand_prob)
        else:
            return "Others", 1.0
    
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
    
    def td_2_1d(self,arr):
        new = []
        for i in arr:
            new.append((i[0]*self.img_w)+(i[1]*self.img_w*self.img_h))
        return np.array(new)
    
    def data2array(self, results1, results2):
        face_rot = self.get_head_rot(results1)
        lh= np.zeros((21, 2))
        rh= np.zeros((21, 2))
        rr = np.zeros((2, 3))
        lr = np.zeros((2, 3))
        if results2.multi_hand_landmarks:
          for hand_landmarks, handedness in zip(results2.multi_hand_landmarks,results2.multi_handedness):
              side = str(handedness.classification[0].label[0:]).lower()
              hand = np.array([[res.x,res.y,res.z] for res in hand_landmarks.landmark])#.flatten()
              #val_h = self.td_2_1d(hand)
              val_h = self.normalize(hand)
              if side=='left':
                  lh = val_h[:,:2]
                  rr = np.array([hand[9],hand[0]])
              else:
                  rh = val_h[:,:2]
                  lr = np.array([hand[9],hand[0]])
        return face_rot, lh, rh, np.concatenate((rr,lr), axis=0)

class MultiPerDetector():
    def __init__(self,rp, num_person, hand_model="", head_model="", fing_model="", seq_len=10, classify_thresh=0.4):
        #dir_path = os.path.dirname(os.path.realpath(__file__))
        #print(dir_path+"/"+per_path)
        self.frame_num=0
        self.seq_len = seq_len
        self.avg_len = 2
        self.num_person = num_person
        self.tracker = EuclideanDistTracker(max_id=num_person)
        self.keys = {}
        self.fingers_rot = {}
        self.pd = PoseDetector(hand_model=hand_model, head_model=head_model, fing_model=fing_model, seq_len=seq_len, avg_len=self.avg_len, classify_thresh=classify_thresh)
        for i in range(self.num_person):
            self.keys[i] = deque(maxlen=self.seq_len*self.avg_len)
            self.fingers_rot[i] = deque(maxlen=self.seq_len*self.avg_len)
        self.rp = rp
        
    def det_per(self,frame, min_area_per=0.1):
        w,h = frame.shape[:2]
        bboxs = self.rp.detect_person(frame.copy())
        filtered = []
        padd = 20
        for rect in bboxs:
            x1,y1 = max(0,rect[0]-padd), max(0,rect[1]-padd)
            x2,y2 = min(w,rect[2]+padd), min(h,rect[3]+padd)
            rect1 = [x1,y1,x2,y2]
            area = (rect1[1]-rect1[3])*(rect1[0]-rect1[2])
            if area > (w*h*min_area_per):
                filtered.append(rect1)
        objects_bbs_ids = self.tracker.assign(filtered)
        #print(bboxs,objects_bbs_ids)
        return objects_bbs_ids
    
    def multi_per_gesture(self,image,viz=False):
        gestures = {}
        for i in range(self.num_person):
            gestures[i] = []
        objects_bbs_ids = self.det_per(image)
        #print(objects_bbs_ids)
        for idn in objects_bbs_ids:
            bbox = objects_bbs_ids[idn]
            if objects_bbs_ids[idn]!=[]:
                person = image[bbox[1]:bbox[3],bbox[0]:bbox[2]]
                results1, results2 = self.pd.mediapipe_detection(person)
                face_rot, lh, rh, rot = self.pd.data2array(results1, results2)
                key = np.concatenate([face_rot.flatten(),lh.flatten(),rh.flatten()])
                if viz:
                    per = self.pd.draw_landmarks(person, results1, results2)
                    image[bbox[1]:bbox[3],bbox[0]:bbox[2]] = per
                    cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255,255,255), thickness=2)
                    cv2.putText(image, str(idn), (bbox[0], int(bbox[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            else:
                key = np.zeros(86)
                rot = np.zeros((4,3))
            if self.frame_num%2==0:
                self.keys[idn].append(key)
                self.fingers_rot[idn].append(rot)
            if len(self.keys[idn])==self.seq_len*self.avg_len: #i%10==0 and 
                gest, prob = self.pd.classify(self.keys[idn], self.fingers_rot[idn])
                finger_count, fing_prob = self.pd.fingersUp(self.keys[idn])
                gestures[idn] = [gest, prob, idn, bbox, str(finger_count),fing_prob] #gest, prob, id, bbox, finger_count
                self.keys[idn] = deque(maxlen=self.seq_len*self.avg_len)
                self.fingers_rot[idn] = deque(maxlen=self.seq_len*self.avg_len)
        self.frame_num = self.frame_num+1
        return image, gestures
    
    def combine_multi_per_gest(self, gestures):
        gesture_final = "Others"
        gest_prob_max = 0
        fing_final = "Others"
        fing_prob_max = 0
        for i in gestures:
            if gestures[i]!=[]:
                if gest_prob_max<gestures[i][1] and gestures[i][0]!="Others":
                    gesture_final = gestures[i][0]
                    gest_prob_max = gestures[i][1]
                    
                if fing_prob_max<gestures[i][5] and gestures[i][4]!="Others":
                    fing_final = gestures[i][4]
                    fing_prob_max = gestures[i][5]
                
        return [gesture_final,gest_prob_max,0,[0,0,0,0],fing_final]