#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import time
import argparse

import cv2
import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gesture_name', '-n', type=str,
                        default='unknown_gesture', help='name of gesture')
    parser.add_argument('--duration', '-d', type=float,
                        default=5., help='duration of video to record')
    parser.add_argument('--dir_path', '-f', type=str,
                        default='../data/gesture_examples/videos/', help='path to directory in which video'
                        'will be saved')
    args = parser.parse_args()

    camera_device = cv2.VideoCapture(0)
    if camera_device.isOpened():
        print('Successfully opened camera device')
    else:
        print('Could not open camera device!')

    if os.path.isfile(os.path.join(args.dir_path, args.gesture_name)+'.mp4'):
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

    frame_width = int(camera_device.get(3))
    frame_height = int(camera_device.get(4))
    fps = int(camera_device.get(5))
    camera_fourcc = int(camera_device.get(6))

    # Note: any fourcc (codec) causes a warning for mp4 format; can be ignored.
    video_record_device = cv2.VideoWriter(os.path.join(args.dir_path,
                                                       args.gesture_name)+'.mp4',
                                          camera_fourcc,
                                          fps, (frame_width, frame_height))
    end_time = time.time() + args.duration

    print('Recording video...')
    while (time.time() < end_time):
        success, frame = camera_device.read()

        if success:
            video_record_device.write(frame)

        cv2.imshow('Camera Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print('Finished recording video.')
    camera_device.release()
    video_record_device.release()
    cv2.destroyAllWindows()
