#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mdr_common_states'],
   package_dir={'mdr_common_states': 'ros/src/mdr_common_states'}
)

distutils.core.setup(**d)
