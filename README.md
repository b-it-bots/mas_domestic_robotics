# mas_domestic_robotics

## Summary

This repository contains various core domestic robotics functionalities developed by the Autonomous Systems group at Hochschule Bonn-Rhein-Sieg.

The code in this repository is mostly ROS-based and is developed in a robot-independent manner. Robot-dependent code and/or configuration files are hosted in robot-specific repositories, e.g. [mas_cob](https://github.com/b-it-bots/mas_cob) for our Care-O-bot.

## Packages

The functionalities in this repository are organised based on the main capabilities a domestic robot needs to possess. Each of these has its own ROS metapackage, which further includes a set of packages related to that particular functionality. A rough summary of the contents of these packages contents can be found below:

* `mdr_navigation` - Mapping and autonomous navigation
* `mdr_manipulation` - Grasp planner
* `mdr_hri` - Visualization, sound vocalization, graphical user interfaces
* `mdr_msgs` - ROS messages for all the mas_domestic_robotics packages
* `mdr_speech` - Speech recognition and NLP
* `mdr_perception` - Object recognition, people detection and recognition.

### Planning

The planning metapackage is where most of the high-level functionalites of our robots are implemented. We are developing our robots as skill-based agents; the planning metapackage includes *action* and *scenario* metapackages because of that. The action metapackage is further divided into metapackages for actions related to particular capabilities, while the scenario metapackage includes domain files and/or state machines for various scenarios (e.g. RoboCup@Home tasks, lab demos, and so forth). In principle, scenarios are built by integrating actions together, which is the main benefit of the skill-based agent framework.

### Naming Conventions

All packages in our domestic code base start with the `mdr_` suffix; this stands for `mas_domestic_robotics`, which is the name of the top-level directory/ROS metapackage.


## Getting started

b-it-bots members can use [these instructions](https://github.com/b-it-bots/dev-env#setup) to setup a complete development environment for all our robots.

For external users, the following instructions should get you a working system:

1. Setup a catkin workspace

  ```
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
  wstool init src
  wstool merge -t src https://github.com/b-it-bots/mas_domestic_robotics/mas-domestic.rosinstall
  ```
2. Get the code and dependencies

  ```
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

  ```

3. Building your code

  ```
  catkin build
  ```

## License

This project is licensed under the GPLv3 License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Thanks to the many generations of b-it-bots@Home members. You can see a list of contributors [here](https://github.com/b-it-bots/mas_domestic_robotics/graphs/contributors).
* MAS staff and professors who have provided their advice and support
