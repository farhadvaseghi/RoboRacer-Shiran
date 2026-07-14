"""sim_opponent_test.launch.py — validate the LiDAR opponent detector in sim.

Runs the sim (gym + moving opponent + EKF + RViz) and our opponent_detector on
the sim /scan, publishing to /perception/opp_odom (NOT /opp_racecar/odom, to
avoid clashing with the simulator's ground-truth on that topic). Compare the two
to check accuracy:

    ros2 launch roboracer_camera sim_opponent_test.launch.py
    # in another terminal:
    ros2 topic echo /perception/opp_odom     # our estimate
    ros2 topic echo /opp_racecar/odom        # ground truth (sim cheat)

On the real car this same node instead publishes /opp_racecar/odom directly
(see perception_camera.launch.py), replacing the cheat with real perception.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('roboracer_estimation'), 'launch', 'sim.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),

        Node(package='roboracer_camera', executable='opponent_detector',
             name='opponent_detector', output='screen',
             parameters=[{'output_frame': 'ego_racecar/odom',
                          'odom_topic': '/perception/opp_odom'}]),
    ])
