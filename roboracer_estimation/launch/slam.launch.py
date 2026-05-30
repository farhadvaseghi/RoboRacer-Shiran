import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('roboracer_estimation'),
        'config', 'slam_params.yaml')

    gym_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROBORACER_MAP_NAME', 'solid_oval_track'),
        SetEnvironmentVariable('ROBORACER_SIM_CONFIG_NAME', 'sim_moving_obstacle.yaml'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gym_launch),
            launch_arguments={
                'use_rviz': 'true',
                'publish_map_odom_tf': 'false',
            }.items(),
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),
    ])
