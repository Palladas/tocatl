from glob import glob
from setuptools import find_packages, setup
name = 'tocatl_kinematics'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])],
      tests_require=['pytest'], install_requires=['setuptools'], zip_safe=True,
      maintainer='Matthew Hutchinson', maintainer_email='matthewhutchinson@ucla.edu',
      description='Recovered TOCATL leg geometry and kinematics', license='MIT',
      entry_points={'console_scripts': ['geometry_reference = tocatl_kinematics.legacy_geometry:main']})
