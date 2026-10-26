from glob import glob
from setuptools import find_packages, setup
name = 'tocatl_diagnostics'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])],
      tests_require=['pytest'], install_requires=['setuptools'], zip_safe=True,
      maintainer='Matthew Hutchinson', maintainer_email='matthewhutchinson@ucla.edu',
      description='Motor inventory and optional CAN telemetry', license='MIT',
      entry_points={'console_scripts': ['motor_map = tocatl_diagnostics.motor_map:main', 'motor_telemetry = tocatl_diagnostics.cybergear_telemetry:main']})
