import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('roboracer_estimation')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Nav2 navigation and path-planning stack. Vehicle control is launched
    # separately from roboracer_control/controller.launch.py.
    # navigation_launch.py does NOT start map_server or AMCL — the sim already
    # provides /map and the gym's perfect odometry gives us map→odom for free.
    nav2_stack = GroupAction([
        # Keep Nav2's internal controller output away from the simulator's
        # teleoperation input. Only roboracer_control publishes vehicle commands.
        SetRemap(src='/plan', dst='/nav2/plan_raw'),
        SetRemap(src='/cmd_vel', dst='/navigation/cmd_vel'),
        SetRemap(src='/cmd_vel_smoothed', dst='/navigation/cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),
    ])

    path_relay = Node(
        package='roboracer_estimation',
        executable='path_relay_node',
        name='path_relay',
        output='screen',
    )

    return LaunchDescription([
        nav2_stack,
        path_relay,
    ])
