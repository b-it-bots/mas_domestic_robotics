# mdr_behaviors_msgs

## Introduction

A package for behavior-related messages and services

## Messages

## Services

### mdr_behaviors_msgs/Pickup

Request:
* ``geometry_msgs/PointStamped position``

Response:
* ``bool success``

### mdr_behaviors_msgs/Place

Request:
* ``geometry_msgs/PointStamped position``

Response:
* ``bool success``

### mdr_behaviors_msgs/CleanTable

Request:
* ``arm_navigation_msgs/Shape area``
* ``geometry_msgs/PointStamped area_center``
* ``geometry_msgs/PointStamped sponge_position``

Response:
* ``bool success``
