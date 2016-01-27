## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    #https://docs.python.org/2/distutils/examples.html
    py_modules=['SerialAPI', 'SerialAPI_bat'], #this installs the individual modules to the PYTHON_PATH
    package_dir={'': 'src'},
)

setup(**setup_args)

