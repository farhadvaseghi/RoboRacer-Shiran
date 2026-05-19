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

    obstacle_controller = Node(
        package='roboracer_perception',
        executable='moving_obstacle_controller',
        name='moving_obstacle_controller',
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'sim': 'true',
                'map_name': 'solid_oval_track',
                'config_name': 'sim_moving_obstacle.yaml',
            }.items(),
        ),
        obstacle_controller,
    ])
