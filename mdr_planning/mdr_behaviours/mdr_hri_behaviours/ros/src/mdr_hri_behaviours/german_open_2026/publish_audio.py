#!/usr/bin/env python
import rospy
from audio_common_msgs.msg import AudioData
import subprocess

rospy.init_node('audio_publisher_arecord')
pub = rospy.Publisher('audio', AudioData, queue_size=10)

# Parameters
CHUNK = 1024  # bytes per read
RATE = 16000
CHANNELS = 1

# Start arecord process
cmd = ['arecord', '-f', 'S16_LE', '-c', str(CHANNELS), '-r', str(RATE)]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)

rospy.loginfo("Publishing audio from arecord...")

while not rospy.is_shutdown():
    data = proc.stdout.read(CHUNK)
    if not data:
        break
    msg = AudioData(data=data)
    pub.publish(msg)