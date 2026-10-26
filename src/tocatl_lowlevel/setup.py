from glob import glob
from setuptools import find_packages, setup
name = 'tocatl_lowlevel'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])],
      tests_require=['pytest'], install_requires=['setuptools'], zip_safe=True,
      maintainer='Matthew Hutchinson', maintainer_email='matthewhutchinson@ucla.edu',
      description='Basic motor control and command validation', license='MIT',
      entry_points={'console_scripts': ['motor_controller = tocatl_lowlevel.motor_controller:main', 'validate_calibration = tocatl_lowlevel.calibration:main']})
