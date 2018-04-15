from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_object_recognition'],
    package_dir={'': 'ros/src'}
)

setup(**d)
