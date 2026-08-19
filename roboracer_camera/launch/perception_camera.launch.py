"""perception_camera.launch.py — REAL-CAR camera perception (ZED must be running).

Starts our camera stack on hardware:
  * static TF base_link -> camera_scan (for depth_to_scan)
  * depth_to_scan     : ZED depth -> /camera_scan (Nav2 costmap obstacle source)
  * person_detector   : YOLO person -> /perception/persons  (needs ultralytics+GPU)
  * emergency_brake   : /drive_nav -> /drive, zeroed when a person is in the path

Run the ZED wrapper separately first (see DEPLOY.md). For the AEB to sit in the
drive path, launch the autonomous stack with use_aeb:=true so the controller
publishes /drive_nav instead of /drive:

    ros2 launch roboracer_estimation autonomous_real.launch.py use_aeb:=true map:=...
    ros2 launch roboracer_camera perception_camera.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cam_share = get_package_share_directory('roboracer_camera')
    params = os.path.join(cam_share, 'config', 'camera_params.yaml')

    depth_topic = LaunchConfiguration('depth_topic')
    info_topic = LaunchConfiguration('info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic',
                              default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('info_topic',
                              default_value='/zed/zed_node/rgb/camera_info'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_to_camera_scan',
             arguments=['--x', '0.270', '--y', '-0.005', '--z', '0.155',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_scan']),

        Node(package='roboracer_camera', executable='depth_to_scan',
             name='depth_to_scan', output='screen',
             parameters=[{'depth_topic': depth_topic, 'info_topic': info_topic,
                          'scan_topic': '/camera_scan', 'scan_frame': 'camera_scan'}]),

        Node(package='roboracer_camera', executable='person_detector',
             name='person_detector', output='screen',
             parameters=[params, {'depth_topic': depth_topic, 'info_topic': info_topic}]),

        # LiDAR opponent -> /perception/opp_odom (the MPC controller's input).
        Node(package='roboracer_camera', executable='opponent_detector',
             name='opponent_detector', output='screen',
             parameters=[{'output_frame': 'odom', 'odom_topic': '/perception/opp_odom'}]),

        Node(package='roboracer_camera', executable='emergency_brake',
             name='emergency_brake', output='screen',
             parameters=[params]),
    ])
