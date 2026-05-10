"""RoboRacer perception stack launch file.

Usage
-----
# Simulation — starts sim + perception + RViz in one command (default)
ros2 launch roboracer_perception perception.launch.py

# Simulation — explicit
ros2 launch roboracer_perception perception.launch.py sim:=true

# Real hardware — full stack (LiDAR + ZED 2i + fusion)
ros2 launch roboracer_perception perception.launch.py sim:=false

Sim mode topic mapping
----------------------
f1tenth_gym_ros publishes the ego LiDAR on /ego_racecar/scan.
The lidar_processor node subscribes to /scan.
A remapping bridges the two so no Python code changes are needed when
moving to real hardware (where the HOKUYO driver publishes /scan directly).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('roboracer_perception')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'roboracer_sim.rviz')
    cone_map_yaml = os.path.join(pkg_share, 'maps', 'cone_track.yaml')

    sim = LaunchConfiguration('sim')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description=(
            'true  → simulation mode: f1tenth_gym_ros + LiDAR pipeline + RViz  '
            'false → real hardware: LiDAR + ZED 2i + fusion'
        ),
    )

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='oval_track',
        description='Map name (stem, no extension) from roboracer_perception/maps/.',
    )

    declare_config_name = DeclareLaunchArgument(
        'config_name',
        default_value='sim.yaml',
        description='Sim config filename from f1tenth_gym_ros/config/.',
    )

    # ------------------------------------------------------------------
    # Static TF publishers — always present (sim and real)
    #
    # In sim the gym_bridge already publishes:
    #   map → ego_racecar/base_link → ego_racecar/laser
    # Our static publishers add the ADDITIONAL frames used by
    # Estimation/Control (rear_axle, front_axle) relative to base_link.
    # On real hardware these cover all frames.
    # ------------------------------------------------------------------
    # base_link origin = rear axle, ground level (REP-105 / f1tenth convention).
    # All offsets confirmed against the working f1tenth hardware bringup.
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

    # rear_axle = base_link (identity transform — published for teammate convenience)
    tf_rear_axle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_rear_axle',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'rear_axle',
        ],
    )

    tf_front_axle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_front_axle',
        arguments=[
            '--x', '0.324', '--y', '0.0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'front_axle',
        ],
    )

    # ------------------------------------------------------------------
    # LiDAR processor — always present.
    #
    # Both sim and real hardware publish the scan on /scan, so no
    # remapping is needed.  The scan frame_id differs (ego_racecar/laser
    # in sim, laser on real hardware) but the node uses msg.header
    # so it is frame-agnostic.
    # ------------------------------------------------------------------
    lidar_processor = Node(
        package='roboracer_perception',
        executable='lidar_processor_node',
        name='lidar_processor',
        parameters=[params_file],
        output='screen',
    )

    cone_tracker = Node(
        package='roboracer_perception',
        executable='cone_tracker_node',
        name='cone_tracker',
        parameters=[params_file, {
            'input_topic': '/perception/cones',
            'odom_topic': '/ego_racecar/odom',
            'map_frame': 'ego_racecar/odom',
        }],
        output='screen',
        condition=IfCondition(sim),
    )

    # ------------------------------------------------------------------
    # Simulation-only group
    # ------------------------------------------------------------------

    # f1tenth_gym_ros bridge (without its built-in RViz — we provide ours)
    gym_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('f1tenth_gym_ros'), 'launch',
                'gym_bridge_launch.py',
            ])
        ]),
        launch_arguments={
                'use_rviz': 'false',
                'map_name': LaunchConfiguration('map_name'),
                'config_name': LaunchConfiguration('config_name'),
            }.items(),
        condition=IfCondition(sim),
    )

    # RViz with our comprehensive config (sim only)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(sim),
    )

    sim_group = GroupAction(
        condition=IfCondition(sim),
        actions=[gym_bridge, rviz],
    )

    # ------------------------------------------------------------------
    # Real-hardware-only group
    # ------------------------------------------------------------------

    # HOKUYO UST-10LX via Ethernet (default 192.168.0.10)
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'ip_address': '192.168.0.10',
            'ip_port': 10940,
            'frame_id': 'laser',
            'angle_min': -2.3562,
            'angle_max':  2.3562,
        }],
        remappings=[('/scan', '/scan')],
        output='screen',
        condition=UnlessCondition(sim),
    )

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py',
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'base_frame': 'zed_camera_link',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=UnlessCondition(sim),
    )

    camera_processor = Node(
        package='roboracer_perception',
        executable='camera_processor_node',
        name='camera_processor',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(sim),
    )

    perception_fusion = Node(
        package='roboracer_perception',
        executable='perception_fusion_node',
        name='perception_fusion',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(sim),
    )

    real_group = GroupAction(
        condition=UnlessCondition(sim),
        actions=[
            tf_laser, tf_zed, tf_rear_axle, tf_front_axle,
            urg_node, zed_wrapper, camera_processor, perception_fusion,
        ],
    )

    # ------------------------------------------------------------------
    # Assemble
    # ------------------------------------------------------------------
    return LaunchDescription([
        declare_sim,
        declare_map_name,
        declare_config_name,

        # LiDAR pipeline
        lidar_processor,
        cone_tracker,

        # Mode-specific groups
        sim_group,
        real_group,
    ])
