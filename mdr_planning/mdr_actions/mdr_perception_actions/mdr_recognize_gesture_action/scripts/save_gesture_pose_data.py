#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import time
import argparse

import cv2
import numpy as np
from openpose import pyopenpose as op


float_formatter = "{:.3f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--video_device', '-v', type=str,
                        default='0', help='The video device. For a camera,'
                        'provide its id, for e.g. 0. For a video file, provide'
                        'its path')
    parser.add_argument('--gesture_name', '-n', type=str,
                        default='unknown', help='name of gesture')
    parser.add_argument('--duration', '-d', type=float,
                        default=5., help='Duration of gesture. For a video'
                        'file, this is set to the length of the video.')
    parser.add_argument('--dir_path', '-f', type=str,
                        default='../data/gesture_examples/', 
                        help='path to directory in which npy data'
                        'will be saved')
    parser.add_argument('--display', '-D', type=int, default=1, 
                        help='whether to display frames while processing')
    args = parser.parse_args()

    opWrapper = op.WrapperPython()
    opWrapper.configure({"model_folder" : '/home/afaisal/.models/openpose_models/'})
    opWrapper.start()

    try:
        camera_device = cv2.VideoCapture(int(args.video_device))
    except ValueError:
        camera_device = cv2.VideoCapture(args.video_device)

    if camera_device.isOpened():
        print('Successfully opened camera device')
    else:
        print('Could not open camera device!')
        sys.exit()

    end_time = time.time() + args.duration
    full_poses = []

    if os.path.isfile(os.path.join(args.dir_path, args.gesture_name)+'.npy'):
        confirm = raw_input('The file in this path already exists and will be'
                            ' overwritten! Would you like to continue? [Y/n]')
        if confirm == '' or confirm.lower() == 'y':
            pass
        elif confirm.lower() == 'n':
            print('Aborting')
            sys.exit()
        else:
            print('Invalid option! Aborting')
            sys.exit()

    print('\nExtracting gesture pose data...')
    while(time.time() < end_time): 
        success, frame = camera_device.read()

        datum = op.Datum()
        datum.cvInputData = frame
        opWrapper.emplaceAndPop([datum])

        frame = datum.cvOutputData

        if datum.poseKeypoints.shape == ():
            print('Body probably too close to adequately estimate pose.' 
                  ' Please move away from the camera.')

        else:
            if datum.poseKeypoints.shape[0] > 1:
                datum.poseKeypoints = datum.poseKeypoints[0, :, :][np.newaxis]
            if bool(args.display):
                frame = cv2.putText(frame, 'Recording...', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 0, 255), 2, cv2.LINE_AA)

            pose_array_relevant_dims = datum.poseKeypoints[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15, 16, 17, 18], :]
            full_poses.append(pose_array_relevant_dims)

        if bool(args.display):
            cv2.imshow('frame', frame) 
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break
     
    print('Finished recording, exiting program...')
    if bool(args.display):
        camera_device.release() 
        cv2.destroyAllWindows() 

    full_pose_array = np.vstack(full_poses)
    print('Saving pose data (selected dims) to npy binary files...')
    np.save(os.path.join(args.dir_path, args.gesture_name), full_pose_array)
