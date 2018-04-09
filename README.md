# mas_domestic_robotics

## Summary

This repository contains various core domestic robotics functionalities developed by the Autonomous Systems group at Hochschule Bonn-Rhein-Sieg.

The code in this repository is mostly ROS-based and is developed in a robot-independent manner.

## Repository Organisation

The functionalities in this repository are organised based on the main capabilities a domestic robot needs to possess - navigation, manipulation, perception, speech, and planning. Each of these has its own ROS metapackage, which further includes a set of packages related to that particular functionality.

The planning metapackage is where most of the high-level functionalites of our robots are implemented. We are developing our robots as skill-based agents, so the planning metapackage includes *action* and *scenario* metapackages. The action metapackage is further divided into metapackages for actions related to particular capabilities, while the scenario metapackage includes domain files and/or state machines for various scenarios (e.g. RoboCup@Home tasks, lab demos, and so forth). In principle, scenarios are built by integrating actions together, which is the main benefit of the skill-based agent framework.

## Naming conventions

All packages in our domestic code base start with the `mdr_` suffix; this stands for `mas_domestic_robotics`, which is the name of the top-level directory/ROS metapackage.

## High-Level Directory Structure

```
mas_domestic_robotics
|____mas_domestic_robotics
|    |    CMakeLists.txt
|    |____package.xml
|    |
|____mdr_manipulation
|    |____mdr_manipulation
|    |    |    CMakeLists.txt
|    |    |____package.xml
|    |____mdr_simple_grasp_planner
|    |    |    CMakeLists.txt
|    |    |    package.xml
|    |    |____ros
|____mdr_msgs
|    |____mdr_msgs
|    |    |    CMakeLists.txt
|    |    |____package.xml
|    |____mdr_behavior_msgs
|    |____mdr_manipulation_msgs
|    |____mdr_monitoring_msgs
|    |____mdr_perception_msgs
|    |____mdr_speech_msgs
|____mdr_navigation
|    |____mdr_navigation
|    |    |    CMakeLists.txt
|    |    |____package.xml
|    |____mdr_2dnav
|    |____mdr_2dslam
|____mdr_perception
|    |____mdr_perception
|    |    |    CMakeLists.txt
|    |    |____package.xml
|    |____mdr_object_recognition
|    |____mdr_perception_libs
|    |____mdr_perception_selectors
|____mdr_planning
|    |____mdr_planning
|    |    |    CMakeLists.txt
|    |    |____package.xml
|    |____mdr_actions
|    |____mdr_rosplan_interface
|    |____mdr_scenarios
|____mdr_speech
     |____mdr_speech
     |    |    CMakeLists.txt
     |    |____package.xml
     |____mdr_question_matching
```

## Setup

A fairly detailed explanation about the setup of this repository can be found [here](mdr_docs/setup.md).
