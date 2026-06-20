"""
slam_mapping.launch.py — Build a map from scratch with SLAM (no prior map, no opponent).

The simulator still needs a map image for physics/lidar ray-casting, but the known
map is intentionally NOT published to ROS topics. slam_toolbox builds the map from
ego lidar scans and publishes it itself.

Usage
-----
ros2 launch roboracer_estimation slam_mapping.launch.py
ros2 launch roboracer_estimation slam_mapping.launch.py map_name:=solid_oval_track

Save the finished map
---------------------
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_track
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _make_bridge_node(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)

    pkg_gym = get_package_share_directory('f1tenth_gym_ros')
    pkg_est = get_package_share_directory('roboracer_estimation')

    sim_config = os.path.join(pkg_gym, 'config', 'sim.yaml')
    map_path = os.path.join(pkg_est, 'maps', map_name)

    return [Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, {
            'map_path': map_path,
            # slam_toolbox owns the map->odom TF; bridge must not publish it
            'publish_map_odom_tf': False,
        }],
    )]


def generate_launch_description():
    pkg_gym = get_package_share_directory('f1tenth_gym_ros')
    pkg_est = get_package_share_directory('roboracer_estimation')

    slam_params = os.path.join(pkg_est, 'config', 'slam_params.yaml')

    ego_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command([
            'xacro ', os.path.join(pkg_gym, 'launch', 'ego_racecar.xacro'),
        ])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_gym, 'launch', 'gym_bridge.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='solid_oval_track',
            description='Sim map used for physics/lidar only — not published to ROS',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),
        OpaqueFunction(function=_make_bridge_node),
        ego_rsp,
        rviz_node,
        slam_node,
    ])
