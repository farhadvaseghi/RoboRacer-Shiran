cmake_minimum_required(VERSION 3.8)
project(roboracer_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Custom messages
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs
)

# ---------------------------------------------------------------------------
# Python package
# rosidl_generator_py (called above) sets PYTHON_INSTALL_DIR via
# ament_python_install_package, so we reuse that variable to land our source
# files in the same site-packages directory as the generated message classes.
# ---------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(
  DIRECTORY roboracer_perception/
  DESTINATION ${PYTHON_INSTALL_DIR}/roboracer_perception
  FILES_MATCHING PATTERN "*.py"
)

# ---------------------------------------------------------------------------
# Node entry-point executables  (ros2 run roboracer_perception <name>)
# ---------------------------------------------------------------------------
install(PROGRAMS roboracer_perception/wall_opponent_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wall_opponent_detector
)
install(PROGRAMS roboracer_perception/camera_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_processor_node
)
install(PROGRAMS roboracer_perception/sim_camera_walls.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_camera_walls
)
install(PROGRAMS roboracer_perception/moving_obstacle_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moving_obstacle_controller
)
install(PROGRAMS roboracer_perception/solid_wall_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME solid_wall_visualizer
)
install(PROGRAMS roboracer_perception/solid_wall_scan_highlighter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME solid_wall_scan_highlighter
)
install(PROGRAMS roboracer_perception/teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_key
)

# ---------------------------------------------------------------------------
# Launch / config / map assets
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
