#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_tactile_grasp', 'mdr_tactile_grasp_common'],
    package_dir={'mdr_tactile_grasp': 'ros/src/mdr_tactile_grasp',
                 'mdr_tactile_grasp_common': 'common/src/mdr_tactile_grasp_common'}
)

setup(**d)
