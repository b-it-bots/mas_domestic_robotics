#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mdr_lab_demo'],
   package_dir={'mdr_lab_demo': 'ros/src/mdr_lab_demo'}
)

distutils.core.setup(**d)
