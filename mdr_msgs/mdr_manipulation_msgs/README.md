# mdr_manipulation_msgs

## Introduction

A package for manipulation-related messages and services

## Messages

## Services

### mdr_manipulation_msgs/Grasp

Request:
* ``geometry_msgs/PointStamped position``

Response:
* ``int64 result``

### mdr_manipulation_msgs/GetBottleState

Response:
* ``int8 state``

Possible values for ``state``:
* ``int8 NO_BOTTLE_GRASPED = 1``
* ``int8 EMPTY = 2``
* ``int8 HALF_FULL = 3``
* ``int8 FULL = 4``
