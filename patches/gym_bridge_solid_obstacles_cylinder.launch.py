# MIT License

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    perception_launch = os.path.join(
        get_package_share_directory('roboracer_perception'),
        'launch',
        'perception.launch.py',
    )
    wall_visualizer = Node(
        package='roboracer_perception',
        executable='solid_wall_visualizer',
        name='solid_wall_visualizer',
        output='screen',
    )
    wall_scan_highlighter = Node(
        package='roboracer_perception',
        executable='solid_wall_scan_highlighter',
        name='solid_wall_scan_highlighter',
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'sim': 'true',
                'map_name': 'solid_oval_track_obstacles',
            }.items(),
        ),
        wall_visualizer,
        wall_scan_highlighter,
    ])
