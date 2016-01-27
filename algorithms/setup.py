## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    #packages=['algorithms'],
    py_modules=['BathymCreate','SingleBeam','Transformations','curvefit','Controllers','Lines','Boustrophedon','TSP',
                'AStar','ASV_KF','Kalman','myGP','CholFact'], #this installs the individual modules to the PYTHON_PATH
    package_dir={'': 'scripts'},
)

setup(**setup_args)

