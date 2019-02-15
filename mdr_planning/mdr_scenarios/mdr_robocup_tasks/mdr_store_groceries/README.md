# mdr_store_groceries

## Goal

In this task, the robot has to correctly identify and manipulate objects at different heights, grouping them by category and likelihood. This test focuses on the detection and recognition of objects and their features, as well as object
manipulation.

## Subtasks involved

- Open door (not yet implemented)
- Listen for speech command to start task (Currently disabled)
- Inspect cupboard and table
- Recognize and classify objects
- Grasping objects
- Placing objects

### Setup

1. Place location: Third shelf of the only open cupboard in the lab.
2. Start position: The robot can start in any position and oritentation. (But it has to be localized first)
3. Objects: Objects are placed on the table next to the shelf.

## Running the task

The store groceries task can only be run on the lucy account at the moment. The first step is localizing Lucy. You can localize Lucy using rviz. This localization step can be done either from running rviz on lucy or from your computer.

1. To run rviz in your computer, first export the ROS master from Lucy. On your terminal, write:

    ```
    export ROS_MASTER_URI=http://192.168.50.201:11311/
    rviz
    ```
    or on a terminal on Lucy, just run:

    ```
    rviz
    ```

2. Load the config file with the hsr model. (If you do not have it, ask to one of the team leaders)
3. Use the 2D Pose Estimate to localize the robot in the map.

4. Now that the robot is localized, you can run the store groceries task. In one terminal, you need to run the SSD Keras Image detector:
    ```
    roslaunch ssd_keras_ros coco_image_detection_server.launch cloud_topic:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
    ```

    In a different terminal:

    ```
    roslaunch mas_hsr_store_groceries store_groceries.launch
    ```
