cmake_minimum_required(VERSION 3.8)

project(f1tenth_simulator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Library source files
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})

# Link library
target_link_libraries(${PROJECT_NAME})

# Declare ROS2 dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# Node executables
file(GLOB NODE_SRC_FILES node/*.cpp)
foreach(_node_file ${NODE_SRC_FILES})
  get_filename_component(_node_name ${_node_file} NAME_WE)
  add_executable(${_node_name} ${_node_file})
  target_link_libraries(${_node_name} ${PROJECT_NAME})
  ament_target_dependencies(${_node_name}
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    ackermann_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
  )
  install(TARGETS ${_node_name}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install parameter files
install(FILES params.yaml
  DESTINATION share/${PROJECT_NAME})

# Install maps
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps)

# Install URDF/xacro files
install(FILES racecar.xacro
  DESTINATION share/${PROJECT_NAME})

# Install RViz config
install(FILES launch/simulator.rviz
  DESTINATION share/${PROJECT_NAME}/launch)

# Install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
