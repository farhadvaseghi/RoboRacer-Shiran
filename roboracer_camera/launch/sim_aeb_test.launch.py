"""sim_aeb_test.launch.py — validate the AEB reflex in simulation (no camera).

Brings up the sim (gym + opponent + RViz + EKF), drives the ego forward with a
constant command on /drive_nav, fakes a person on /perception/persons (the sim
opponent, or a scripted approacher), and inserts emergency_brake between the two:

    sim_test_drive ─ /drive_nav ─► emergency_brake ─ /drive ─► gym (ego)
    sim_person_publisher ─ /perception/persons ─► emergency_brake

Expected: the ego drives, and when a person enters the danger corridor
(< stop_distance) the car halts; it resumes when the corridor clears.

    ros2 launch roboracer_camera sim_aeb_test.launch.py
    ros2 launch roboracer_camera sim_aeb_test.launch.py person_mode:=scripted
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cam_share = get_package_share_directory('roboracer_camera')
    params = os.path.join(cam_share, 'config', 'camera_params.yaml')

    sim_launch = os.path.join(
        get_package_share_directory('roboracer_estimation'), 'launch', 'sim.launch.py')

    person_mode = LaunchConfiguration('person_mode')

    return LaunchDescription([
        DeclareLaunchArgument('person_mode', default_value='opponent',
                              description="'opponent' (sim opp car) or 'scripted'"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),

        Node(package='roboracer_camera', executable='sim_test_drive',
             name='sim_test_drive', output='screen',
             parameters=[params]),

        Node(package='roboracer_camera', executable='sim_person_publisher',
             name='sim_person_publisher', output='screen',
             parameters=[params, {'mode': person_mode}]),

        Node(package='roboracer_camera', executable='emergency_brake',
             name='emergency_brake', output='screen',
             parameters=[params]),
    ])
