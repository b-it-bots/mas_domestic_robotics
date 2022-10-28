Components Running on Startup
=============================
These are the components running on system startup. After connecting to the HSR robot we can also view all the services using 
the command below.

.. code-block:: bash
   
   systemctl

Description of services:

- hsr_dmp
    The service launches the dynamic motion primitive (DMP) server service. This service is useful during learning a DMP.
- hsr_image_detection
    The service launches the image detection node. Generally used of detecting objects in a domestic environments.  
- hsr_map
    The service launches the map_server by providing a map that is present on the hsr robot. The map is loaded from this path:
    ``/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_environments/$ROBOT_ENV/map.yaml``

    The path is set based on the environment variable ``$ROBOT_ENV``.
    
- hsr_move_arm_action
    The service launches the move arm action. This service is useful for moving the arm (or the whole body) to named
    targets (e.g. Folded configuration).
- hsr_move_arm_joints_action
    The service launches the move arm joints action. This service is useful for moving a particular joint to a specified joint values
    (e.g. Controlling head_pan_joint, arm_roll_joint, wrist_roll_joint, etc).
- hsr_move_base_action
    The service launches the move base action server. An action for moving a robot to a specified location.
- hsr_move_base_client
    The service is useful for launching the move base action client.
- hsr_move_forward_action
    The service launches the move forward action. An action for moving a robot's base forward for a given number of seconds
    at a given speed. This action does not take obstacles into account.
- hsr_perceive_plane_action
    The service launches the perceive plane action server. An action for recognizing objects on a plane. 
- hsr_perceive_plane_client
    The service is useful for launching the perceive plane action client.
- hsr_pickup_action
    The service launches the pickup action server. An action for moving the arm to a specified pose and closing 
    the gripper around it.
- hsr_pickup_client
    The service is useful for launching the pickup action client.
- hsr_place_action
    The service launches the place action server. An action for moving the arm to a specified pose and opening the gripper there.
- hsr_place_client
   The service is useful for launching the place action client.
- hsr_sound_vocalisation
    The service launches the sound vocalisation. This service can be used for speech synthesis and sound generation.
- hsr_teleop
    The service launches the teleoperation. This service can be used for controlling the robot with joystick.
- hsr_turn_base_to_action
    The service launches the turn base action. An action to turn a robot base to face a given orientation.


