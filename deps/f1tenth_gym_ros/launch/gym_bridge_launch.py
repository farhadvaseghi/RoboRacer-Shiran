# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _make_nodes(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context)
    config_name = LaunchConfiguration('config_name').perform(context)

    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        config_name,
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1

    map_path = os.path.join(
        get_package_share_directory('roboracer_perception'),
        'maps',
        map_name,
    )

    f1tenth_share = get_package_share_directory('f1tenth_gym_ros')

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {'map_path': map_path}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(f1tenth_share, 'launch', 'gym_bridge.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': map_path + '.yaml'},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': False},
        ],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']},
            {'bond_timeout': 4.0},
        ],
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(f1tenth_share, 'launch', 'ego_racecar.xacro')]
        )}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(f1tenth_share, 'launch', 'opp_racecar.xacro')]
        )}],
        remappings=[('/robot_description', 'opp_robot_description')],
    )

    nodes = [rviz_node, bridge_node, nav_lifecycle_node, map_server_node, ego_robot_publisher]
    if has_opp:
        nodes.append(opp_robot_publisher)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name', default_value='oval_track',
            description='Map name (stem, no extension) from roboracer_perception/maps/'),
        DeclareLaunchArgument(
            'config_name', default_value='sim.yaml',
            description='Sim config filename from f1tenth_gym_ros/config/'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Set to false to suppress the built-in RViz instance.'),
        OpaqueFunction(function=_make_nodes),
    ])
