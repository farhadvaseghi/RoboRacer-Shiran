"""
sim.launch.py — Full RoboRacer simulation stack.

Starts the f1tenth gym (solid oval track, two-agent mode), the moving-obstacle
controller for the opponent car, and the full EKF estimation stack in one command.

Usage
-----
ros2 launch roboracer_estimation sim.launch.py
ros2 launch roboracer_estimation sim.launch.py ekf_frequency:=30.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_estimation(context, *args, **kwargs):
    ekf_frequency = float(LaunchConfiguration('ekf_frequency').perform(context))
    if ekf_frequency <= 0.0:
        ekf_frequency = 50.0

    pkg_share = get_package_share_directory('roboracer_estimation')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_sim.yaml')

    adaptive_node = Node(
        package='roboracer_estimation',
        executable='adaptive_covariance_node',
        name='adaptive_covariance_node',
        parameters=[{'is_sim': True, 'use_sim_time': False}],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'frequency': ekf_frequency, 'use_sim_time': False}],
        output='screen',
    )

    return [adaptive_node, ekf_node]


def generate_launch_description():
    gym_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py',
    )

    # moving_obstacle = Node(
    #     package='roboracer_estimation',
    #     executable='moving_obstacle_controller',
    #     name='moving_obstacle_controller',
    #     output='screen',
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_frequency',
            default_value='0.0',
            description='EKF update rate in Hz; 0 = auto (50 Hz in sim)'),

        # Tell gym_bridge_launch.py which map and sim config to use.
        SetEnvironmentVariable('ROBORACER_MAP_NAME', 'solid_oval_track'),
        SetEnvironmentVariable('ROBORACER_SIM_CONFIG_NAME', 'sim_moving_obstacle.yaml'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gym_launch),
            launch_arguments={'use_rviz': 'true'}.items(),
        ),

        # moving_obstacle,  # re-enable for dynamic overtaking experiments

        OpaqueFunction(function=_launch_estimation),
    ])
