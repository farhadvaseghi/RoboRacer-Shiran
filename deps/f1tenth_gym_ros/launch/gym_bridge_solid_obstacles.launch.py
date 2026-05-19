# MIT License

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    perception_launch = os.path.join(
        get_package_share_directory('roboracer_perception'),
        'launch',
        'perception.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'sim': 'true',
                'map_name': 'solid_oval_track_obstacles',
            }.items(),
        ),
    ])
