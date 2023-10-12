from __future__ import print_function
import sys
import operator

import cv2
import numpy as np
from openpose import pyopenpose as op
from mdr_recognize_gesture.pointing_gesture_recognizer import PointingGestureRecognizer


def draw_bbs(bbs, frame):
    for bb in bbs:
        frame = cv2.rectangle(frame, (bb[0], bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0, 255, 0), 3)
    return frame

float_formatter = "{:.3f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

vid = cv2.VideoCapture(0)
width = vid.get(cv2.CAP_PROP_FRAME_WIDTH)
height = vid.get(cv2.CAP_PROP_FRAME_HEIGHT)

poseModel = op.PoseModel.BODY_25
print(op.getPoseBodyPartMapping(poseModel))

bbs = [(75, 75, 50, 50),
       (50, 350, 150, 50),
       (500, 280, 50, 120)]

gesture_recognizer = PointingGestureRecognizer(60., "/home/afaisal/.models/openpose_models/", False)

while(True): 
    ret, frame = vid.read()

    frame = draw_bbs(bbs, frame)
    success, obj_index, frame = gesture_recognizer.get_object_pointed_to(bbs, frame)

    cv2.imshow('frame', frame) 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
 
vid.release() 
cv2.destroyAllWindows() 
