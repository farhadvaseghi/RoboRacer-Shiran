cmake_minimum_required(VERSION 3.8)
project(roboracer_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp tf2 tf2_ros ackermann_msgs nav_msgs geometry_msgs visualization_msgs)

install(TARGETS pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
