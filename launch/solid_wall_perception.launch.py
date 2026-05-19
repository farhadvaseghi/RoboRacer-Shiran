"""Solid-wall oval track with moving obstacle, LiDAR + camera, and RViz.

Pipeline:
  - lidar_processor_pca: detects opponent car cluster from /scan
  - sim_camera_walls:    fakes a ZED feed showing the wall tubes (yellow/blue)
  - camera_processor:    HSV-detects the wall tubes in that fake feed

Usage
-----
ros2 launch roboracer_perception solid_wall_perception.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg         = get_package_share_directory('roboracer_perception')
    params      = os.path.join(pkg, 'config', 'tube_track_params.yaml')
    cam_params  = os.path.join(pkg, 'config', 'perception_params.yaml')
    rviz_cfg    = os.path.join(pkg, 'config', 'roboracer_sim.rviz')

    gym_bridge_base = PathJoinSubstitution([
        FindPackageShare('f1tenth_gym_ros'), 'launch', 'gym_bridge_launch.py',
    ])

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gym_bridge_base),
        launch_arguments={
            'map_name':    'solid_oval_track',
            'config_name': 'sim_moving_obstacle.yaml',
            'use_rviz':    'false',
        }.items(),
    )

    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=[
            '--x', '0.270', '--y', '0.0', '--z', '0.110',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
    )

    lidar_processor = Node(
        package='roboracer_perception',
        executable='wall_opponent_detector',
        name='wall_opponent_detector',
        output='screen',
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

    tf_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed',
        arguments=[
            '--x', '0.270', '--y', '-0.005', '--z', '0.155',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link',
        ],
    )

    sim_camera = Node(
        package='roboracer_perception',
        executable='sim_camera_walls',
        name='sim_camera_walls',
        output='screen',
    )

    camera_processor = Node(
        package='roboracer_perception',
        executable='camera_processor_node',
        name='camera_processor',
        parameters=[cam_params],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        bridge,
        tf_laser,
        tf_zed,
        lidar_processor,
        obstacle_controller,
        wall_visualizer,
        wall_scan_highlighter,
        sim_camera,
        camera_processor,
        rviz,
    ])
