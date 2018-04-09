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

A high-level directory tree of the repository can be found [here](docs/high-level-dir-structure.md).

## Setup

A fairly detailed explanation about the setup of this repository can be found [here](docs/setup.md).
