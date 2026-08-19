"""
estimation.launch.py — RoboRacer state estimation stack

Spins up two nodes:
  1. adaptive_covariance_node  — pre-processes sensor covariances
  2. ekf_node (robot_localization) — fuses pre-processed inputs

Launch arguments
----------------
is_sim        true  → gym/sim mode  (subscribes /ego_racecar/odom, uses ekf_sim.yaml)
              false → real hardware (subscribes ZED + VESC topics, uses ekf_real.yaml)
use_sim_time  true  → nodes consume /clock from the sim; false → wall clock
ekf_frequency EKF update rate in Hz.  0 = auto-select (50 Hz sim, 100 Hz real).

Examples
--------
# Simulation (defaults)
ros2 launch roboracer_estimation estimation.launch.py

# Real hardware
ros2 launch roboracer_estimation estimation.launch.py \
    is_sim:=false use_sim_time:=false

# Force a custom EKF rate in sim
ros2 launch roboracer_estimation estimation.launch.py ekf_frequency:=30.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# OpaqueFunction — runs at launch time with resolved argument values
# ---------------------------------------------------------------------------

def _launch_nodes(context, *args, **kwargs):
    is_sim = LaunchConfiguration('is_sim').perform(context).lower() in ('true', '1', 'yes')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')

    ekf_frequency = float(LaunchConfiguration('ekf_frequency').perform(context))
    if ekf_frequency <= 0.0:
        ekf_frequency = 50.0 if is_sim else 100.0

    pkg_share = get_package_share_directory('roboracer_estimation')
    ekf_config = os.path.join(
        pkg_share, 'config', 'ekf_sim.yaml' if is_sim else 'ekf_real.yaml')

    adaptive_node = Node(
        package='roboracer_estimation',
        executable='adaptive_covariance_node',
        name='adaptive_covariance_node',
        parameters=[{
            'is_sim': is_sim,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ekf_node reads its full config from the YAML; the dict below only
    # overrides frequency and use_sim_time so both can be set at launch time
    # without duplicating every YAML key here.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            ekf_config,
            {
                'frequency': ekf_frequency,
                'use_sim_time': use_sim_time,
            },
        ],
        output='screen',
    )

    return [adaptive_node, ekf_node]


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'is_sim',
            default_value='true',
            description='true = gym/sim mode, false = real hardware'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true = consume /clock from sim, false = wall clock'),
        DeclareLaunchArgument(
            'ekf_frequency',
            default_value='0.0',
            description='EKF update rate in Hz; 0 = auto (50 sim / 100 real)'),
        OpaqueFunction(function=_launch_nodes),
    ])
