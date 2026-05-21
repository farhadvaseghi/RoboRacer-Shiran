# MIT License

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    base_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py',
    )

    obstacle_controller = Node(
        package='roboracer_perception',
        executable='moving_obstacle_controller',
        name='moving_obstacle_controller',
        output='screen',
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
        SetEnvironmentVariable('ROBORACER_MAP_NAME', 'solid_oval_track'),
        SetEnvironmentVariable('ROBORACER_SIM_CONFIG_NAME', 'sim_moving_obstacle.yaml'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
        obstacle_controller,
        wall_visualizer,
        wall_scan_highlighter,
    ])
