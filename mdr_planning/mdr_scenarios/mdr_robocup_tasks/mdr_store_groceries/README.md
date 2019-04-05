# mdr_store_groceries

## Goal

In this task, the robot has to correctly identify and manipulate objects at different heights, grouping them by category and likelihood. This test focuses on the detection and recognition of objects and their features, as well as object
manipulation.

## Subtasks involved

- Open door (not yet implemented)
- Listen for speech command to start task (Currently disabled. Note: this is not a requirement for the task and it is only used for demonstration purposes.)
- Inspect cupboard and table
- Recognize and classify objects
- Grasp objects
- Place objects

## Running the task

We provide and example of what needs to be run for the store groceries task. The first step is localizing the robot. You can localize it using rviz. This localization step can be done from running rviz from your computer.

1. To run rviz in your computer, first export the ROS master from the robot. On your terminal, write:

    ```
    export ROS_MASTER_URI=http://<Robot_IP>:11311/
    rviz
    ```

2. Load the config file with the robot model.

3. Use the 2D Pose Estimate to localize the robot in the map.

4. Now that the robot is localized, you can run the store groceries task. In one terminal, you need to run the SSD Keras Image detector:
    ```
    roslaunch ssd_keras_ros coco_image_detection_server.launch cloud_topic:=/point_cloud_topic_name
    ```

    Then, in a different terminal:

    ```
    roslaunch mdr_store_groceries store_groceries.launch
    ```
