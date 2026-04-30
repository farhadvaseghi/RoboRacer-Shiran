# MIT License

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROBORACER_MAP_NAME', 'solid_oval_track_obstacles'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
    ])
