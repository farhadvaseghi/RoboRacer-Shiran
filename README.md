# F1TENTH Autonomous Racing – SLAM & Navigation

This branch (`planner-setup`) contains the configuration and launch files required to map the environment and run the autonomous navigation stack.

---

## 🛠️ Installation & Setup

Before running the nodes, you must install the Navigation 2 dependencies and build the workspace.

### Install Nav2 Dependencies

    sudo apt update
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

### Build the Workspace

    cd ~/ros2_ws
    colcon build --symlink-install --packages-select f1tenth_simulator
    source install/setup.bash

---

## 🗺️ Phase 1: Mapping (SLAM)

Use the following commands to generate a map of the race track.

### 1. Start the Simulator
    source install/setup.bash
    ros2 launch f1tenth_simulator simulator.launch.py

### 2. Launch SLAM Node (in a new terminal)
    source install/setup.bash
    ros2 launch f1tenth_simulator slam.launch.py

### 3. Drive the Car (in a new terminal)

Use the keyboard to cover the entire track until the map is complete in RViz.
    
    ros2 run f1tenth_simulator keyboard

### 4. Save the Map

    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/RoboRacer-Shiran/maps/my_track
