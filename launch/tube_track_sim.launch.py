"""Launch the f1tenth simulator with the tube-boundary track map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'map_name': 'tube_track'}.items(),
        ),
    ])
