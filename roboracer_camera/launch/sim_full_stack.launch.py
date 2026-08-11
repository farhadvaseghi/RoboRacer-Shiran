"""sim_full_stack.launch.py — full sim stack.

  * sim.launch             — gym (ego + moving opponent) + EKF + RViz
  * navigation.launch      — Nav2 planner (SMAC) -> /control/plan
  * controller.launch      — custom pure_pursuit controller -> /drive
  * wall_opponent_detector — YOUR LiDAR detector (roboracer_perception),
                             /scan -> /perception/{walls,opponent,detections}
                             (visualisation only)

    ros2 launch roboracer_camera sim_full_stack.launch.py

Fixes now live in the teammate files (root causes, not launch workarounds):
  - ekf_sim.yaml: EKF fuses gym pose ONLY -> no longer lags/under-reports during
    motion, so the controller no longer overshoots goals into walls.
  - moving_obstacle_controller.py: opponent oval turn corrected 20 m -> 60 m so it
    loops the (resized) track instead of driving off the end into the wall.
  - gym opponent-reset remapped off /goal_pose (see sim.launch) so the RViz
    "2D Goal Pose" navigates the EGO, not the opponent.

NOTE: the SMAC planner takes ~40 s to configure at startup — wait until the log
prints "Configured plugin GridBased" before sending the first 2D Goal Pose.
The opponent_odom_adapter / lidar_gap_safety nodes still exist in this package
but are intentionally NOT launched.
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

        # YOUR perception detector — visualisation of walls/opponent only.
        Node(package='roboracer_perception', executable='wall_opponent_detector',
             name='wall_opponent_detector', output='screen'),
    ])
