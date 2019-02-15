# `mdr_behaviours`

## Table of contents

1. [Summary](#summary)
2. [Organisation](#organisation)
3. [Dependencies](#dependencies)
4. [Implemented behaviours](#implemented-behaviours)
    1. [mdr_navigation_behaviours](#mdr_navigation_behaviours)
    2. [mdr_perception_behaviours](#mdr_perception_behaviours)
    3. [mdr_manipulation_behaviours](#mdr_manipulation_behaviours)
    4. [mdr_knowledge_behaviours](#mdr_knowledge_behaviours)

## Summary

Most domestic scenarios include common components, such as going to certain locations or manipulating objects in a certain way, The `mdr_behaviours` metapackage groups together such functionalities, thereby facilitating the reusability of components when building complex robot scenarios and minimising unnecessary code repetition in the process.

The name `behaviours` indicates that the components included in this package make robots behave in a certain manner. In this context, we can define an *atomic behaviour* as a specific way of accomplishing something in a certain way. Under this definition, "picking the closest object from a surface" is a specific robot behaviour that differs from "picking a random object from the surface". Atomic behaviours on their own may not make much sense, but when used in conjunction with other atomic behaviours, they give rise to *complex behaviours* that allow robots to perform complex tasks. For instance, combining the atomic behaviours
1. going to the table
2. turning towards the table
3. locating objects on the table
4. picking the closest object from the table
5. going to a cupboard
6. placing the object in the cupboard

allows us to define a simple pick-and-place behaviour (as done [here](../mdr_scenarios/mdr_demos/mdr_demo_simple_pick_and_place/config/pick_and_place_sm.yaml)).

As defined here, the term behaviour has a small semantic difference with our use of the term action as defined in [`mdr_actions`](../mdr_actions), which is why actions and behaviours are kept separately. We use the term action in the classical planning sense, namely an action can be thought of as a function with a certain set of preconditions that, if executed successfully, leads to a given set of effects; on the other hand, a behaviour precedes an action and, roughly speaking, controls what the action does. In the pick-and-place example above, the behaviour of "picking the closest object from the table" precedes a grasping action and defines that a robot should try to grasp the closest rather than a randomly selected object on the table, thereby allowing the robot to make more "optimal" decisions.

All behaviours included here implement the [`scenario state base class`](https://github.com/b-it-bots/mas_execution_manager/blob/master/ros/src/mas_execution_manager/scenario_state_base.py) in `mas_execution_manager` and interact with the action clients in `mdr_actions`. Each behaviour is implemented in a dedicated Python script.

## Organisation

The metapackage is split into the following packages:
* [`mdr_navigation_behaviours`](mdr_navigation_behaviours): A collection of navigation-related behaviours
* [`mdr_perception_behaviours`](mdr_perception_behaviours): Behaviours for perceiving the world in a certain way
* [`mdr_manipulation_behaviours`](mdr_manipulation_behaviours): A group of behaviours for manipulating objects in the world
* [`mdr_knowledge_behaviours`](mdr_knowledge_behaviours): Functionalities that let a robot use its knowledge about the world in order to make informed decisions

## Dependencies

* `mas_execution_manager`
* `mas_knowledge_base`
* The actions and action clients in the `mdr_actions` metapackage

## Implemented behaviours

### `mdr_navigation_behaviours`

* `move_base`: Moves through a set of predefined navigation waypoints, updating the location of the robot in the knowledge base during the process

### `mdr_perception_behaviours`

* `perceive_planes`: Scans a plane (or a set of planes) assuming that the robot's camera is already oriented towards the plane(s)

### `mdr_manipulation_behaviours`

* `pick_closest_from_surface`: Given a surface and a set of objects on it, finds the closest object with respect to the base_link frame of the robot
* `place`: Places an object on a surface whose name starts with a given prefix
* `place_based_on_category`: Places an object on a surface that contains objects of the same category (if objects of the same category are present on multiple surfaces, chooses the surfaces with the highest number of objects of that category)
* `throw_object_in`: Throws an object in a given throwing target

### `mdr_knowledge_behaviours`

* `check_empty_surface`: Checks whether a given surface is empty based on the most recent knowledge in the knowledge base
