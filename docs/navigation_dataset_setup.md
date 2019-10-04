# Setup instructions

## Lucy: Change map

- Use lucy account:
    ```
    su - lucy
    password:
    ```

- Change ROBOT_ENV variable in home/lucy/robot.env

    Example:
    ```
    export ROBOT_ENV=brsu-c069
    ```
- Copy map folder to `etc/opt/tmc/robot/conf.d`

- Update map path in `etc/opt/tmc/robot/docker-hsr.user`
    
- Restart Lucy after changing ROBOT_ENV


### Connect to Lucy

- Connect to the same router as Lucy

- ssh into Lucy
    ```
    ssh lucy@192.168.50.201
    ```

### Rviz

- Get hrs.rviz file
- Launch rviz. Go to File/Open Config and select hsr.rviz
- Select Map Topic under the option Map
- Select option Range Sensor to visualize ranger sensors
- Save rviz configuration (Ctr+S)

Localize the robot:

- Click on 2D Pose Estimate
- Select on the map an estimated pose for the robot
- Move the robot around to a unique feature in the map. Visualize the sensor data and improve the pose as needed

### Configure your PC to use Lucy's ros master
- Check to which port to connect by exporting the variable
```
echo $ROS_MASTER_URI
```
- Set the ROS_MASTER_URI variable using Lucy's IP and the port showed in variable ROS_MASTER_URI
```
export ROS_MASTER_URI=http://192.168.50.201:1131
```
- Check your computer's IP address (ip addr)
- Set the ROS_IP variable with your computer's IP address. Example:
```
export ROS_IP=192.168.50.89
```

### Use script to save map poses to a file
- Run the script:
```
rosrun mas_navigation_tools save_base_map_poses_to_file
```

- Move the robot to the desired pose, enter the name of the pose and press enter
- Once all poses have been recorded, press Ctr+C
- Poses will be stored in file `navigation_goals.yaml`
- Secure copy the file to your computer
```
scp username@from_host:file.txt /local/directory/
```

    Example:
```
scp lucy@hsr.local:/home/lucy/ros/kinetic/src/mas_navigation_tools/navigation_goals.yaml navigation_goals.yaml
```

- Copy the new `navigation_goals.yaml` to `mdr_environments` into the corresponding map folder. For example, if the poses were recorded using the `hbrs-ground-floor-cartographer`, copy the `navigation_goals.yaml` into `hbrs-ground-floor-cartographer`

- verify that the ROBOT_ENV variable is set to the corresponding map name
```
echo $ROBOT_ENV
```
- If necessary, set it to the correct value
```
export ROBOT_ENV=hbrs-ground-floor-cartographer
```

### Visualize recorded poses

- Check that the `pose_visualizer.launch` searches for environments using the package `mdr_environments`
```
roscd mas_navigation_tools
vim ros/launch/pose_visualizer.launch
```

    The pose_description_file parameter should look like this:
```
<param name="pose_description_file" value="$(find mdr_environments)/$(arg robot_env)/navigation_goals.yaml" />
```

- Launch the pose_visualizer
```
roslaunch mas_navigation_tools pose_visualizer.launch
```

### Change patrol scenario goals to use newly recorded poses
- Go to the `mdr_demo_patrol` package
```
roscd mdr_demo_patrol
```
- Open the `patrol_sm.yaml` file and write the desired goals in the arguments
```
vim config/patrol_sm.yaml
```

Example:
```
...
arguments:
           - argument:
               name: destination_locations
               value: [pose_1, pose_2, pose_3]
```
- Launch the demo
```
roslaunch mdr_demo_patrol patrol.launch
```
