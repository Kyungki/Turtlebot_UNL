# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rqt_turtlebot_houston'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_turtlebot_houston']
)

setup(**setup_args)