"""sim_full_stack.launch.py — full sim stack driven by YOUR perception.

One command that proves your processed data is used end-to-end:
  * sim.launch          — gym (ego + moving opponent) + EKF + RViz
  * navigation.launch   — Nav2 planner (SMAC) -> /control/plan
  * controller.launch   — custom pure_pursuit + MPC overtaking (use_aeb=true)
  * opponent_detector   — YOUR LiDAR detector -> /perception/opp_odom

The MPC now consumes /perception/opp_odom (your detection), NOT the sim's
ground-truth /opp_racecar/odom. So the car follows/overtakes the opponent based
on YOUR perception. Compare the two topics to validate accuracy:

    ros2 launch roboracer_camera sim_full_stack.launch.py
    ros2 topic echo /perception/opp_odom     # yours (drives the MPC)
    ros2 topic echo /opp_racecar/odom         # sim ground truth (for comparison)

(person -> AEB is wired via use_aeb; add sim_person_publisher to trigger it.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    est = get_package_share_directory('roboracer_estimation')
    ctrl = get_package_share_directory('roboracer_control')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(est, 'launch', 'sim.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(est, 'launch', 'navigation.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(ctrl, 'launch', 'controller.launch.py'))),

        Node(package='roboracer_camera', executable='opponent_detector',
             name='opponent_detector', output='screen',
             parameters=[{'output_frame': 'ego_racecar/odom',
                          'odom_topic': '/perception/opp_odom'}]),
    ])
