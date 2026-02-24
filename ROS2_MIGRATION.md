# F1TENTH Simulator - ROS1 to ROS2 (Humble) Migration Guide

This document describes how to migrate this project from ROS1 to ROS2 Humble on Ubuntu 22.04.

## Prerequisites

```bash
# Install ROS2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS2 setup
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Step 1: Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/f1tenth/f1tenth_simulator.git
cd ~/ros2_ws
```

## Step 2: Install Dependencies

ROS2 Humble requires:
```bash
sudo apt install -y \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-map-server \
  ros-humble-joy \
  ros-humble-tf2-geometry-msgs \
  ros-humble-visualization-msgs
```

## Step 3: Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Migration Changes Made

### 1. package.xml
- Changed format to ROS2 (format="2")
- Replaced `catkin` buildtool with `ament_cmake`
- Changed `build_depend`/`run_depend` to `build_depend`/`exec_depend`
- Removed `message_generation` (not needed in ROS2)
- Removed obsolete dependencies (interactive_markers, cv_bridge, image_transport)

### 2. CMakeLists.txt
- Changed `cmake_minimum_required` to 3.8+ (ROS2 requirement)
- Replaced `find_package(catkin ...)` with individual `find_package()` calls for each dependency
- Replaced `catkin_package()` with `ament_package()`
- Updated install directives to use ROS2 conventions
- Uses `ament_target_dependencies()` instead of `target_link_libraries()`

### 3. Node Files (C++)

Key pattern changes:

**Old (ROS1):**
```cpp
#include <ros/ros.h>

class RacecarSimulator {
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "simulator");
    RacecarSimulator rs;
    ros::spin();
}
```

**New (ROS2):**
```cpp
#include <rclcpp/rclcpp.hpp>

class RacecarSimulator : public rclcpp::Node {
public:
    RacecarSimulator() : Node("simulator") {
        this->declare_parameter<double>("wheelbase", 0.3302);
        auto wheelbase = this->get_parameter("wheelbase").as_double();
        
        sub_ = this->create_subscription<message_type>(
            "topic", rclcpp::QoS(10),
            std::bind(&RacecarSimulator::callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<message_type>("topic", rclcpp::QoS(10));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RacecarSimulator::timer_callback, this));
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacecarSimulator>());
    rclcpp::shutdown();
}
```

### 4. Launch Files

**Old (launch/simulator.launch - XML):**
```xml
<launch>
  <node pkg="f1tenth_simulator" name="simulator" type="simulator" />
</launch>
```

**New (launch/simulator.launch.py - Python):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_simulator',
            executable='simulator',
            name='f1tenth_simulator',
        ),
    ])
```

## Step 4: Run the Simulator

```bash
cd ~/ros2_ws
source install/setup.bash

# Launch the simulator
ros2 launch f1tenth_simulator simulator.launch.py

# In another terminal, drive the car
source install/setup.bash

# List available topics
ros2 topic list

# Manual control (keyboard mode - press 'k')
# Or use random walker (press 'r')
```

## Remaining Nodes to Migrate

All nodes follow the same pattern. Key files:

1. **simulator.cpp** - Main simulation loop
   - Change: `ros::NodeHandle` → `rclcpp::Node` inheritance
   - Change: `ros::Timer` → `create_wall_timer()`
   - Change: Parameter loading pattern

2. **behavior_controller.cpp** - Safety/mux logic
   - Change: Subscription callbacks use `std::bind`
   - Change: Parameter loading with `declare_parameter()`

3. **mux.cpp** - Command multiplexer
   - Change: Same patterns as above

4. **keyboard.cpp** - Keyboard driver
   - Change: Terminal I/O stays similar, just wrap in rclcpp::Node

5. **random_walk.cpp** - Example planner
   - Change: Subscription/publisher callbacks

## Important Notes

### ROS1 to ROS2 API Mapping

| ROS1 | ROS2 |
|------|------|
| `ros::NodeHandle n` | Inherit from `rclcpp::Node` |
| `n.getParam()` | `this->declare_parameter()` + `this->get_parameter()` |
| `n.advertise<T>(topic, queue)` | `this->create_publisher<T>(topic, QoS(queue))` |
| `n.subscribe(topic, queue, &Class::cb, this)` | `this->create_subscription<T>(topic, QoS(queue), std::bind(&Class::cb, this, std::placeholders::_1))` |
| `n.createTimer(duration, &Class::cb, this)` | `this->create_wall_timer(duration, std::bind(&Class::cb, this))` |
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::Time::now()` | `this->now()` or `rclcpp::Clock().now()` |

### TF2 Changes (Mostly Compatible)

TF2 API is largely the same. Main changes:
- TF2 is now part of ROS2
- You may need to add `#include <rclcpp/time_source.hpp>` for clock sync
- `tf2_ros::TransformBroadcaster` constructor takes a `Node*` instead of `NodeHandle`

### Map Server

ROS1: `map_server` package
ROS2: `nav2_map_server` package

Topics remain the same (`/map`), but launch invocation is different:

```python
# ROS2 launch file
map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{'yaml_filename': map_file}],
)
```

## Troubleshooting

### "Could not find a package configuration file"

```bash
# Ensure all dependencies are installed
rosdep install --from-paths src --ignore-src -r -y

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### Compilation errors on node files

- Ensure `#include <rclcpp/rclcpp.hpp>` is first include
- Check that callbacks use `std::bind` with placeholders
- Verify parameter declarations before `get_parameter()` calls

### RViz not showing robot model

```bash
# Install xacro if needed
sudo apt install -y ros-humble-xacro

# Launch racecar model explicitly
ros2 run xacro xacro $(ros2 pkg find f1tenth_simulator)/racecar.xacro | \
  ros2 param set /robot_description -
ros2 run robot_state_publisher robot_state_publisher
```

## Next Steps

1. Migrate remaining node files using the pattern above
2. Create Python launch files for each scenario
3. Test sensor messages (LiDAR `/scan`) on ROS2
4. Validate TF2 tree in RViz2
