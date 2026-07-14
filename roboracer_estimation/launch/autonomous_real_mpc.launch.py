"""autonomous_real_mpc.launch.py — REAL-CAR autonomy with the custom MPC controller.

Phase-2 (guide.md §9): instead of Nav2's Regulated Pure Pursuit, use the team's
custom pure_pursuit + MPC overtaking, fed by the real LiDAR opponent detector.
This is what makes the processed LiDAR contribute ON HARDWARE.

Chain:
  Nav2 (localization + planner only; its controller output is isolated)
      └─ /plan ─► path_relay ─► /control/plan ─┐
  map_odom_relay ─ /odometry/map ──────────────┤
  opponent_detector ─ /perception/opp_odom ────┼─► pure_pursuit + MPC ─ /drive_nav
                                               │        └─► emergency_brake ─► /drive
  (t_stack.sh drivers provide /scan /odom TF /mux)

⚠ UNVALIDATED ON HARDWARE — structurally complete, needs an on-car check.
Control-domain: coordinate before racing. First run capped ~0.8 m/s; joystick
overrides (mux prio 100) and the AEB are the safety nets. Keep autonomous_real
(Nav2 RPP) as the safe fallback — this launch does not touch it.

    ros2 launch roboracer_estimation autonomous_real_mpc.launch.py map:=~/rr_maps/track2.yaml
    # then set the amcl init pose and send a Nav2 goal, as in guide.md §5.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    est = get_package_share_directory('roboracer_estimation')
    ctrl = get_package_share_directory('roboracer_control')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(est, 'config', 'nav2_params_real.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'rr_maps', 'track2.yaml')
    ctrl_params = os.path.join(ctrl, 'config', 'controller_params_real.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map,
                                    description='SLAM map .yaml to navigate on')
    params_arg = DeclareLaunchArgument('params_file', default_value=default_params,
                                       description='Nav2 params (real-car)')

    # Nav2 for map_server + amcl + planner. Its controller still runs but its
    # velocity output is remapped away, and /plan is diverted to path_relay.
    nav2 = GroupAction([
        SetRemap(src='/plan', dst='/nav2/plan_raw'),
        SetRemap(src='/cmd_vel', dst='/navigation/cmd_vel'),
        SetRemap(src='/cmd_vel_smoothed', dst='/navigation/cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'True',
            }.items(),
        ),
    ])

    map_odom = Node(
        package='roboracer_camera', executable='map_odom_relay',
        name='map_odom_relay', output='screen',
        parameters=[{'map_frame': 'map', 'base_frame': 'base_link',
                     'odom_topic': '/odom', 'output_topic': '/odometry/map'}])

    path_relay = Node(
        package='roboracer_estimation', executable='path_relay_node',
        name='path_relay', output='screen',
        parameters=[{'use_forward_oval_route': False,
                     'raw_plan_topic': '/nav2/plan_raw',
                     'odom_topic': '/odometry/map'}])

    opponent = Node(
        package='roboracer_camera', executable='opponent_detector',
        name='opponent_detector', output='screen',
        parameters=[{'output_frame': 'map', 'odom_topic': '/perception/opp_odom'}])

    controller = Node(
        package='roboracer_control', executable='pure_pursuit_controller',
        name='pure_pursuit_controller', output='screen',
        parameters=[ctrl_params, {'use_sim_time': False}],
        remappings=[('/drive', '/drive_nav')])

    aeb = Node(
        package='roboracer_camera', executable='emergency_brake',
        name='emergency_brake', output='screen',
        parameters=[{'drive_in_topic': '/drive_nav', 'drive_out_topic': '/drive',
                     'persons_topic': '/perception/persons',
                     'stop_distance': 1.5, 'release_distance': 1.9,
                     'half_width': 0.35}])

    return LaunchDescription([map_arg, params_arg, nav2, map_odom, path_relay,
                              opponent, controller, aeb])
