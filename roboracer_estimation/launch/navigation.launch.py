import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('roboracer_estimation')
    control_pkg_share = get_package_share_directory('roboracer_control')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    controller_params_file = os.path.join(control_pkg_share, 'config', 'stanley_params.yaml')

    # Nav2 navigation stack only (planner, controller, BT navigator, etc.).
    # navigation_launch.py does NOT start map_server or AMCL — the sim already
    # provides /map and the gym's perfect odometry gives us map→odom for free.
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    # Stanley controller: subscribes to /plan from Nav2 planner and
    # publishes AckermannDriveStamped directly on /drive (what the bridge listens to).
    stanley_controller = Node(
        package='roboracer_control',
        executable='stanley_controller',
        name='stanley_controller',
        parameters=[controller_params_file, {'use_sim_time': False}],
        remappings=[
            ('/odom', '/odometry/filtered'),
            ('/nav', '/drive'),
        ],
        output='screen',
    )

    return LaunchDescription([
        nav2_stack,
        stanley_controller,
    ])
