import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    f1tenth_dir = get_package_share_directory('f1tenth_simulator')

    params_file = os.path.join(f1tenth_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(f1tenth_dir, 'maps', 'levine.yaml')

    # Twist → AckermannDriveStamped converter.
    # Nav2 outputs geometry_msgs/Twist on /cmd_vel; the mux expects
    # AckermannDriveStamped on /nav (nav_drive_topic in params.yaml).
    twist_to_ackermann = Node(
        package='f1tenth_simulator',
        executable='twist_to_ackermann.py',
        name='twist_to_ackermann',
        output='screen',
    )

    # Static identity transform: map → odom.
    # The simulator publishes odom→base_link with the robot's true world
    # position, so the odom frame already coincides with the map frame.
    # No AMCL / SLAM needed — a static transform is sufficient.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # Map server for Nav2 — publishes the pre-built levine map on /map so
    # the global costmap static layer has the full map for path planning.
    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='nav2_map_server',
        parameters=[{'yaml_filename': map_file, 'topic_name': 'map'}],
        output='screen',
    )

    # Lifecycle transitions for the nav2 map server (configure then activate).
    nav2_map_server_configure = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/nav2_map_server', 'configure'],
                output='screen',
            )
        ],
    )

    nav2_map_server_activate = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/nav2_map_server', 'activate'],
                output='screen',
            )
        ],
    )

    # Nav2 navigation stack only (planner, controller, BT navigator, etc.).
    # We use navigation_launch.py — NOT bringup_launch.py — so that Nav2
    # does not launch its own map_server or AMCL (we handle both above).
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        twist_to_ackermann,
        map_to_odom_tf,
        nav2_map_server,
        nav2_map_server_configure,
        nav2_map_server_activate,
        nav2_stack,
    ])
