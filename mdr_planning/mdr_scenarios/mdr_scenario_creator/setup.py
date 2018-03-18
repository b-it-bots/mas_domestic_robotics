#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_scenario_creator'],
   package_dir={'mdr_scenario_creator': 'ros/src/mdr_scenario_creator'}
)

setup(**d)
