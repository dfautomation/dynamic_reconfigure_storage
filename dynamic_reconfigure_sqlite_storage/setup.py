#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamic_reconfigure_sqlite_storage'],
    package_dir={'': 'src'},
    requires=['dynamic_reconfigure_storage_utils', 'rospy']
)

setup(**d)
