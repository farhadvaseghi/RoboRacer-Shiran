import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roboracer_control')
    params_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),

        Node(
            package='roboracer_control',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odom', '/odometry/filtered'),
            ],
            output='screen',
        ),
    ])
