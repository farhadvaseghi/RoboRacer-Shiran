import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    f1tenth_simulator_dir = get_package_share_directory('f1tenth_simulator')
    
    # Default map file
    map_file = os.path.join(f1tenth_simulator_dir, 'maps', 'levine.yaml')
    
    # Parameters file
    params_file = os.path.join(f1tenth_simulator_dir, 'params.yaml')
    
    # Rviz config
    rviz_config = os.path.join(f1tenth_simulator_dir, 'launch', 'simulator.rviz')
    
    # Racecar URDF/Xacro
    racecar_xacro = os.path.join(f1tenth_simulator_dir, 'racecar.xacro')

    rviz_display_topics = [
        '/map',
        '/racecar_sim/update',
        '/scan',
        '/robot_description',
        '/dynamic_viz',
        '/env_viz',
        '/static_viz',
        '/smoothed_path',
        '/tree_lines',
        '/tree_nodes',
        '/waypoint_vis',
        '/path_lines',
        '/converted_scan',
    ]
    topic_list_for_shell = ' '.join(rviz_display_topics)
    
    # ===== Nodes =====
    
    # Joy node (joystick driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    
    # Map server — publishes on /static_map so slam_toolbox can own /map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file, 'topic_name': 'static_map'}],
        remappings=[('/map', '/static_map')],
        output='screen',
    )
    
    # Robot state publisher (publishes robot model from URDF)
    # First, we need to process the xacro file and get the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', racecar_xacro])}],
        output='screen',
    )

    map_server_configure = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                output='screen',
            )
        ],
    )

    map_server_activate = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                output='screen',
            )
        ],
    )
    
    # Main simulator node
    simulator = Node(
        package='f1tenth_simulator',
        executable='simulator',
        name='f1tenth_simulator',
        parameters=[params_file],
        output='screen',
    )
    
    # Mux controller
    mux_controller = Node(
        package='f1tenth_simulator',
        executable='mux',
        name='mux_controller',
        parameters=[params_file],
        output='screen',
    )
    
    # Behavior controller
    behavior_controller = Node(
        package='f1tenth_simulator',
        executable='behavior_controller',
        name='behavior_controller',
        parameters=[params_file],
        output='screen',
    )
    
    # Random walker (example autonomous planner)
    random_walker = Node(
        package='f1tenth_simulator',
        executable='random_walk',
        name='random_walker',
        parameters=[params_file],
        output='screen',
    )
    
    # Keyboard node
    keyboard = Node(
        package='f1tenth_simulator',
        executable='keyboard',
        name='keyboard',
        parameters=[params_file],
        output='screen',
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'rviz2:=debug'],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    )

    rviz_diagnostics = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    (
                        "echo '========== RViz Diagnostics =========='; "
                        "echo 'Configured display topics:'; "
                        f"for topic in {topic_list_for_shell}; do "
                        "echo \"- $topic\"; "
                        "if ros2 topic info \"$topic\" -v > /tmp/rviz_topic_info.txt 2>&1; then "
                        "cat /tmp/rviz_topic_info.txt; "
                        "else "
                        "echo \"[MISSING/UNAVAILABLE] $topic\"; "
                        "cat /tmp/rviz_topic_info.txt; "
                        "fi; "
                        "echo '--------------------------------------'; "
                        "done; "
                        "echo 'TF check (map -> base_link):'; "
                        "if timeout 2s ros2 run tf2_ros tf2_echo map base_link > /tmp/rviz_tf.txt 2>&1; then "
                        "head -n 20 /tmp/rviz_tf.txt; "
                        "else "
                        "echo '[TF ISSUE] map -> base_link'; "
                        "cat /tmp/rviz_tf.txt; "
                        "fi; "
                        "echo '======================================'"
                    ),
                ],
                output='screen',
            )
        ],
    )
    
    # ===== Launch Description =====
    
    ld = LaunchDescription()
    
    # Add all nodes
    ld.add_action(joy_node)
    ld.add_action(map_server)
    ld.add_action(map_server_configure)
    ld.add_action(map_server_activate)
    ld.add_action(robot_state_publisher)
    ld.add_action(simulator)
    ld.add_action(mux_controller)
    ld.add_action(behavior_controller)
    ld.add_action(random_walker)
    ld.add_action(keyboard)
    ld.add_action(rviz)
    ld.add_action(rviz_diagnostics)
    
    return ld
