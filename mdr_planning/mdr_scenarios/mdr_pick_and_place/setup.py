#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mdr_pick_and_place'],
   package_dir={'mdr_pick_and_place': 'ros/src/mdr_pick_and_place'}
)

distutils.core.setup(**d)
