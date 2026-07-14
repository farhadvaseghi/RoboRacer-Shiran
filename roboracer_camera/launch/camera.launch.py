"""camera.launch.py — camera obstacle source for the Nav2 costmap (real car).

Starts, on OUR side only:
  * static TF  base_link -> camera_scan  at the ZED pose (0.270, -0.005, 0.155),
    x-forward / y-left / z-up. The synthetic scan is published in this frame, so
    the node needs NO transform from the ZED wrapper's own TF tree.
  * depth_to_scan node: ZED depth -> /camera_scan LaserScan.

It does NOT start the ZED driver — run the ZED wrapper separately (heavy, and
keeps this launch parseable even before the SDK is installed):

    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

Then confirm the depth + info topic names with `ros2 topic list` and, if they
differ from the defaults below, pass them through:

    ros2 launch roboracer_camera camera.launch.py \
        depth_topic:=/zed2i/zed_node/depth/depth_registered \
        info_topic:=/zed2i/zed_node/rgb/camera_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic')
    info_topic = LaunchConfiguration('info_topic')

    args = [
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
            description='ZED depth image topic (32FC1, metres)'),
        DeclareLaunchArgument(
            'info_topic',
            default_value='/zed/zed_node/rgb/camera_info',
            description='ZED camera_info topic (intrinsics)'),
    ]

    # base_link -> camera_scan : the scan frame, at the physical ZED pose.
    tf_camera_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera_scan',
        arguments=['--x', '0.270', '--y', '-0.005', '--z', '0.155',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_scan'],
        output='screen',
    )

    depth_to_scan = Node(
        package='roboracer_camera',
        executable='depth_to_scan',
        name='depth_to_scan',
        output='screen',
        parameters=[{
            'depth_topic': depth_topic,
            'info_topic': info_topic,
            'scan_topic': '/camera_scan',
            'scan_frame': 'camera_scan',
            'use_sim_time': False,
        }],
    )

    return LaunchDescription(args + [tf_camera_scan, depth_to_scan])
