import rosbag
from datetime import datetime

last_time = None
last_pos = None
count = 0
bag = rosbag.Bag('move_goals.bag')
for topic, msg, t in bag.read_messages():
    if last_time is not None:
        print("\n********************************************************************************")
        print("NUM:", count)
        print("Old Location:\n" + str(last_pos))
        print("Cur Location:\n" + str(msg.goal.target_pose.pose.position))
        print("Duration:\n" + str((t - last_time).to_sec()))
        print("********************************************************************************\n")

    last_time = t
    last_pos = msg.goal.target_pose.pose.position
    count += 1

bag.close()
