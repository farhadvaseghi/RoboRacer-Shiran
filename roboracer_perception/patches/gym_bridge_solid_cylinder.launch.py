import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
    )
    base_launch = os.path.join(launch_dir, 'gym_bridge_launch.py')
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
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
        wall_visualizer,
        wall_scan_highlighter,
    ])
