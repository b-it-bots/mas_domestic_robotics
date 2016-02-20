#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_tactile_tools', 'mdr_tactile_tools_common'],
    package_dir={'mdr_tactile_tools': 'ros/src/mdr_tactile_tools',
                 'mdr_tactile_tools_common': 'common/src/mdr_tactile_tools_common'}
)

setup(**d)
