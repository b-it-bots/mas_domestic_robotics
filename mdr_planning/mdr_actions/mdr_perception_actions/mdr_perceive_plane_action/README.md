# mdr_perceive_plane_action

An action for recognizing objects on a plane.

## Action definition

### Goal:

* ``string plane_config``: Name of the configuration set for setting up the dynamic parameter
server. Each plane should have a set of configurations, and all configuration sets
should be listed in ``config/perceive_plane_configurations.yaml``

### Result:

* ``bool success``
* ``mcr_perception_msgs/ObjectList recognized_objects``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``mdr_object_recognition_mean_circle``
* ``mcr_dynamic_reconfigure_client``
* ``mcr_states``
* ``mcr_perception_msgs``

## Example usage

1. Launch object recognition node (this should launch the entire perception pipeline): ``roslaunch mdr_object_recognition_mean_circle object_recognition.launch``
2. Run the action server: ``roslaunch mdr_perceive_plane_action perceive_plane.launch``
3. Run the client example: ``rosrun mdr_perceive_plane_action perceive_plane_client_test``
