# mdr_find_my_mates

## Goal

In this task, the robot has to correctly report the location of two guests inside a room and describe them to an operator who only knows the guest's names. It is important that the description is unique and not the same for both guests. The location of the guests needs to be described considering the surroundings.

## Bonus reward

In case the robot reports and describes a third person, the team receives bonus points.

## Restrictions

The maximum time for this task is five minutes.

## Subtasks involved

- Localize a guest.
  - Navigate around and recognize sitting or standing persons.
  - Recognize the surrounding and save it together with the person's position.
- Approach a guest.
  - Navigate to a person.
  - Analyze the person's appearance and save it.
  - Start an interview and save the information from the conversation.
- Report to an operator.
  - Navigate to the operator and describe the collected information. 


## Running the task

We provide and example of what needs to be run for the store groceries task. The first step is localizing the robot. You can localize it using rviz. This localization step can be done from running rviz from your computer.

1. To run rviz in your computer, first export the ROS master from the robot. On your terminal, write:

    ```
    export ROS_MASTER_URI=http://<Robot_IP>:11311/
    rviz
    ```

2. Load the config file with the robot model.

3. Use the 2D Pose Estimate to localize the robot in the map.

4. Now that the robot is localized, you can run the find my mates task.

    ```
    roslaunch mdr_find_my_mates.launch
    ```
