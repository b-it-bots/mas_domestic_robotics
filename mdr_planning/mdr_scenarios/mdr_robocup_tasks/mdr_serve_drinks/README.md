# mdr_serve_drinks

Your favorite drinks delivered cold and fresh.

## Goal

The robot has to find all party guest who don't have a drink yet and take their orders. After that the robot goes to the bar, takes the ordered drinks and brings them to the guests. This task focuses mainly on perception and human-robot interaction.
- The robot has to remember people and orders (people change places)
- One of the ordered drinks is missing
  - Bonus points for preemptively informing the guest, that the order is not available
- Instead of grasping and delivering the drinks one by one the robot can optionally use an attached tray
  - The barkeeper places the drinks on the tray
  - The robot must detect if guests take drinks from the tray they didn't order

## Subtasks involved

- Perception
  - Recognize objects (drinks) on a table (bar)
  - Recognize people
  - Recognize if people have drinks
- Speech
  - Having a small conversation / dialogue
- Manipulation
  - Grasping objects
  - Handing over objects

## Running the task

1. To run rviz on your computer, export the robot's `ROS_MASTER_URI` first:
    ```
    export ROS_MASTER_URI=http://<robot_ip>:11311/
    rviz
    ```

2. Load the config file for the robot.

3. Use "2D Pose Estimate" to localize the robot on the map.

4. Run the SSD Keras Image detector (make sure to have the `ROS_MASTER_URI` exported if you use a new terminal):
    ```
    roslaunch ssd_keras_ros coco_image_detection_server.launch cloud_topic:=/point_cloud_topic_name
    ```

5. Launch the serving drinks scenario (remember `ROS_MASTER_URI`)
    ```
    roslaunch mdr_serve_drinks serve_drinks.launch
    ```
