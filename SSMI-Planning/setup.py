## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ssmi_utilities',
              'ssmi_footprints',
              'ssmi_planners',
              'ssmi_mapping'],
    package_dir={'ssmi_utilities': 'src/ssmi_utilities',
                 'ssmi_footprints': 'src/ssmi_footprints',
                 'ssmi_planners': 'src/ssmi_planners',
                 'ssmi_mapping': 'src/ssmi_mapping'}
)
setup(**setup_args)

