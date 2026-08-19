"""
autonomous_real.launch.py — REAL-CAR autonomous navigation (Nav2 end-to-end).

Brings up, in one command:
  * Nav2 localization (map_server + amcl) on a saved SLAM map
  * Nav2 navigation   (planner SMAC-Hybrid + controller RPP + bt_navigator +
                       behaviors + smoother + velocity_smoother + lifecycle)
  * cmd_vel_to_ackermann: converts Nav2's /cmd_vel -> /drive (mux prio 10)

It does NOT start the sensor/drive stack — run ~/t_stack.sh first
(f1tenth_stack: LiDAR + VESC + joystick + mux -> /scan, /odom, TF, /drive in).

Usage
-----
ros2 launch roboracer_estimation autonomous_real.launch.py \
    map:=/home/roboracer/rr_maps/track2.yaml

Set the start pose (amcl) and the goal afterwards — see the guide.
First-run speed is capped to 0.5 m/s in nav2_params_real.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roboracer_estimation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    default_map = os.path.join(
        os.path.expanduser('~'), 'rr_maps', 'track2.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the SLAM map .yaml to navigate on')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Nav2 parameters file (real-car defaults)')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'True',
        }.items(),
    )

    # Nav2 final command (velocity_smoother output) is /cmd_vel.
    cmd_vel_to_ackermann = Node(
        package='roboracer_estimation',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        nav2,
        cmd_vel_to_ackermann,
    ])
