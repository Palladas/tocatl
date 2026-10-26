"""Start TOCATL read-only hardware telemetry."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('tocatl_bringup')) / 'config' / 'hardware.yaml'
    return LaunchDescription([
        Node(package='tocatl_lowlevel', executable='motor_controller',
             name='motor_controller', parameters=[str(config)], output='screen'),
    ])
