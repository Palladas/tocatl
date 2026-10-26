"""Start the lowlevel hardware server; homing waits for a service request."""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = Path(get_package_share_directory('tocatl_bringup')) / 'config'
    venv = os.environ.get('VIRTUAL_ENV')
    python = str(Path(venv) / 'bin' / 'python') if venv else sys.executable
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value=python,
                              description='Python interpreter with the hardware driver dependencies'),
        DeclareLaunchArgument('allow_homing', default_value='true',
                              description='Expose homing services; does not start motion'),
        Node(package='tocatl_lowlevel', executable='motor_controller',
             name='motor_controller', output='screen',
             prefix=[LaunchConfiguration('python_executable')],
             parameters=[str(config_dir / 'hardware.yaml'),
                         str(config_dir / 'homing-hip-knee.yaml'),
                         {'allow_homing': ParameterValue(LaunchConfiguration('allow_homing'), value_type=bool)}]),
    ])
