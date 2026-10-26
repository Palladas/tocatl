from glob import glob
from setuptools import find_packages, setup
name = 'tocatl_bringup'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml'])] + [(f'share/{name}/launch', glob('launch/*.launch.py')), (f'share/{name}/config', glob('config/*.yaml'))],
      tests_require=['pytest'], install_requires=['setuptools'], zip_safe=True,
      maintainer='Matthew Hutchinson', maintainer_email='matthewhutchinson@ucla.edu',
      description='TOCATL launch files and configuration', license='MIT',
      entry_points={'console_scripts': []})
