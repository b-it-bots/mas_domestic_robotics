#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_process_speech_command_action'],
   package_dir={'mdr_process_speech_command_action': 'ros/src/mdr_process_speech_command_action'})

setup(**d)
