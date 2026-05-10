cmake_minimum_required(VERSION 3.8)
project(roboracer_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Custom messages
#
# rosidl_generator_py (invoked below) internally calls
#   ament_python_install_package(${PROJECT_NAME})
# for the generated message Python bindings, which also sets the CMake cache
# variable PYTHON_INSTALL_DIR.  Calling ament_python_install_package a second
# time for the same project would create duplicate CMake targets and fail.
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  DEPENDENCIES std_msgs
)

# ---------------------------------------------------------------------------
# Python node modules
#
# rosidl_generator_py (called above) sets PYTHON_INSTALL_DIR as a CMake
# CACHE INTERNAL variable via ament_python_install_package.  We reuse that
# same variable so our source files land in exactly the same site-packages
# directory as the generated message classes.
# ---------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install our Python source package alongside the rosidl-generated message
# classes so everything is importable as roboracer_perception.<module>.
install(
  DIRECTORY roboracer_perception/
  DESTINATION ${PYTHON_INSTALL_DIR}/roboracer_perception
  FILES_MATCHING PATTERN "*.py"
)

# ---------------------------------------------------------------------------
# Node entry-point executables  (ros2 run roboracer_perception <name>)
# ---------------------------------------------------------------------------
install(PROGRAMS roboracer_perception/lidar_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_processor_node
)
install(PROGRAMS roboracer_perception/camera_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_processor_node
)
install(PROGRAMS roboracer_perception/perception_fusion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_fusion_node
)
install(PROGRAMS roboracer_perception/teleop_key.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_key
)
install(PROGRAMS roboracer_perception/ackermann_to_twist.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ackermann_to_twist
)
install(PROGRAMS roboracer_perception/cone_map_builder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cone_map_builder_node
)
install(PROGRAMS roboracer_perception/cone_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cone_tracker_node
)
install(PROGRAMS roboracer_perception/ackermann_to_autodrive.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ackermann_to_autodrive
)
install(PROGRAMS roboracer_perception/sim_camera.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_camera
)
install(PROGRAMS roboracer_perception/moving_obstacle_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moving_obstacle_controller
)
install(PROGRAMS roboracer_perception/tube_track/lidar_processor_pca.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_processor_pca_node
)
install(PROGRAMS roboracer_perception/tube_track/sim_camera_tubes.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_camera_tubes_node
)
install(PROGRAMS roboracer_perception/tube_track/perception_fusion_pca.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_fusion_pca_node
)

# ---------------------------------------------------------------------------
# Launch, config, and simulation asset files
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  maps
  urdf
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lidar_processor test/test_lidar_processor.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )
  ament_add_pytest_test(test_cone_tracker test/test_cone_tracker.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )
endif()

ament_package()
