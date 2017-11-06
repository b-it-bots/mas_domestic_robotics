# mdr_robocup_tasks

This metapackage contains the state machines for the tasks for all stages performed during RoboCup@Home competitions.

## Packages

* `mdr_robot_inspection` - The scenario for the visual inspection to qualify to the competition


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See [Running on Jenny](#running-on-jenny) for notes on how to run these tests on the robot.

### Prerequisites

In case you don't have them already, make sure you have cloned all the packages mentioned in the package.xml

### Installing

Make sure the corresponding scenario es properly built. You can build an individual package by using:

```
catkin build <package_name>
```

## Running on Jenny

You must always make sure that the three PCs are on the same branch and have been built correctly.
In addition to that, the `cob` aliases need to work between the three PCs without your intervention e.g. having tu input your password.

The speakers and microphone need to be properly connected and should be tested directly on PC3 before starting.

### Robot Inspection Test
In `PC1`:

```shell
roslaunch mdr_robot_inspection robot_inspection.launch
```

```shell
rosrun mdr_robot_inspection robot_inspection
```
