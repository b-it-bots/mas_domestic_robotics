
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mdr_mbot_logger'],
 package_dir={'mdr_mbot_logger': 'ros/src/mdr_mbot_logger'}

)

setup(**d)
