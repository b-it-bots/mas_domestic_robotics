#!/bin/bash
source ~/indigo/devel/setup.bash

export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'

export ROBOT=cob3-1

export ROBOT_ENV=brsu-c069
#export ROBOT_ENV=ipa-apartment

export ROS_MASTER_URI=http://cob3-1-pc1:11311

exec "$@"
