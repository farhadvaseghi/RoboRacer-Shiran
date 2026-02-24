# F1TENTH Simulator - ROS2 Humble Setup & Execution (Ubuntu 22.04)

## Quick Summary

This project has been migrated to **ROS2 Humble** on **Ubuntu 22.04**. Below is the complete step-by-step guide.

---

## Part 1: Environment Setup (One Time)

### 1.1 Install ROS2 Humble

```bash
# Add ROS2 apt repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update and install
sudo apt update
sudo apt install -y ros-humble-desktop

# Add to shell startup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.2 Install Build Tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-xacro

# Initialize rosdep (one time)
sudo rosdep init
rosdep update
```

### 1.3 Install F1TENTH Dependencies

```bash
sudo apt install -y \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-map-server \
  ros-humble-joy \
  ros-humble-tf2-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

---

## Part 2: Workspace Setup

### 2.1 Clone and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/f1tenth/f1tenth_simulator.git
# OR if you're using the local version:
# cp -r /path/to/local/f1tenth_simulator .

# Go back to workspace root
cd ~/ros2_ws

# Install any missing dependencies via rosdep
rosdep install --from-paths src --ignore-src -r -y

# Build with colcon
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2.2 Verify Build

```bash
# Check that executables were built
ls install/lib/f1tenth_simulator/
# Should show: behavior_controller, keyboard, mux, random_walk, simulator

# Check that packages can be found
ros2 pkg find f1tenth_simulator

# List nodes available
ros2 pkg executables f1tenth_simulator
```

---

## Part 3: Running the Simulator

### 3.1 Basic Launch (Full Stack)

```bash
# In a terminal, source workspace
cd ~/ros2_ws
source install/setup.bash

# Launch simulator with all nodes
ros2 launch f1tenth_simulator simulator.launch.py
```

This will start:
- `joy_node` (joystick driver)
- `nav2_map_server` (map provider)
- `robot_state_publisher` (loads URDF)
- `simulator` (main physics/sensor engine)
- `mux_controller` (command multiplexer)
- `behavior_controller` (safety/mode selector)
- `random_walk` (example autonomous planner)
- `keyboard` (keyboard driver)
- `rviz2` (visualization)

**If successful**, you should see:
- RViz2 opens with the map and car visible
- Nodes start with info logs in the terminal
- `/scan`, `/odom`, `/imu` topics publishing

### 3.2 Manual Control (Keyboard)

Once the simulator is running:

**In a new terminal:**
```bash
cd ~/ros2_ws
source install/setup.bash

# Optional: check topics
ros2 topic list
```

**Then in the RViz window or terminal:**
1. Press `k` to toggle **Keyboard mode ON**
   - You should see: `Keyboard turned on`
2. Use keys:
   - `w` = accelerate forward
   - `s` = reverse/accelerate backward
   - `a` = steer left
   - `d` = steer right
   - `space` = brake/stop
3. Watch the car move in RViz and see LiDAR scans update

### 3.3 Autonomous Control (Random Walk)

Instead of keyboard, press `r` to enable the **Random Walker**:
- Car will drive forward at 50% max speed
- Steering angle changes randomly with directional bias
- Reads `/odom` → publishes to `/rand_drive` → forwarded via mux to `/drive`

### 3.4 Reset Car Position

If the car hits a wall or you want to reset:

**Using RViz:**
1. Click "2D Pose Estimate" button in toolbar
2. Click and drag on the map to set desired starting position
3. Car instantly teleports there with zero velocity

**Using command line:**
```bash
ros2 topic pub /pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0, y: 0}, orientation: {w: 1.0}}}' --once
```

---

## Part 4: Monitoring & Debugging

### 4.1 Check Published Topics

```bash
# List all active topics
ros2 topic list

# Watch a specific topic (e.g., /scan)
ros2 topic echo /scan

# Check message rate
ros2 topic hz /scan

# Inspect topic structure
ros2 topic info /scan
```

### 4.2 Check Node Status

```bash
# List all running nodes
ros2 node list

# Inspect a node's subscribers/publishers
ros2 node info /simulator

# View ROS2 graph visually
rqt_graph
```

### 4.3 View TF Tree

```bash
# Print TF tree
ros2 run tf2_tools view_frames.py
# Generates frames.pdf

# Or in RViz: Add → TF
```

### 4.4 LiDAR Visualization in RViz2

1. RViz2 should already be running
2. If not, manually launch:
   ```bash
   ros2 run rviz2 rviz2
   ```
3. Add displays:
   - **Map**: /map (from nav2_map_server)
   - **LaserScan**: /scan (simulated LiDAR)
   - **RobotModel**: loads from /robot_description
   - **TF**: to see frames

---

## Part 5: Customization

### 5.1 Change Map

Edit `launch/simulator.launch.py` and modify the `map_file` line:
```python
map_file = os.path.join(f1tenth_simulator_dir, 'maps', 'your_map.yaml')
```

Available maps in `maps/`:
- `levine.yaml` (default)
- `levinelobby.yaml`
- `columbia.yaml`
- `porto.yaml`
- `mtl.yaml`
- etc.

### 5.2 Modify Simulator Parameters

Edit `params.yaml`:
- `max_speed`: max velocity
- `scan_beams`: LiDAR resolution
- `scan_field_of_view`: LiDAR FOV in radians
- `wheelbase`: distance between front/rear axles
- `update_pose_rate`: simulator update frequency (seconds)

### 5.3 Write Your Own Planner

1. Create a new ROS2 node that:
   - Subscribes to `/scan` (sensor_msgs/LaserScan)
   - Subscribes to `/odom` (nav_msgs/Odometry)  
   - Publishes to a custom drive topic, e.g., `/my_planner`
2. Register in `launch/simulator.launch.py`:
   ```python
   my_planner = Node(
       package='my_package',
       executable='my_planner_node',
       parameters=[params_file],
   )
   ```
3. Add to mux in `params.yaml` (requires code changes to nodes)

---

## Part 6: Troubleshooting

### Issue: "Could not find package..."

**Solution:**
```bash
# Ensure workspace is sourced
source ~/ros2_ws/install/setup.bash

# Rebuild
cd ~/ros2_ws && colcon build --symlink-install

# Check package found
ros2 pkg find f1tenth_simulator
```

### Issue: RViz doesn't show robot model

**Solution:**
```bash
# Manually run robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg find f1tenth_simulator)/racecar.xacro)"
```

### Issue: Nodes fail to start

**Check logs:**
```bash
# Look for error messages in the launch terminal
# Or check individual node:
ros2 run f1tenth_simulator simulator
```

If parameter-related errors, check that `params.yaml` exists in share directory:
```bash
ls $(ros2 pkg prefix f1tenth_simulator)/share/
```

### Issue: Keyboard input not working

```bash
# Verify keyboard node is running
ros2 node list | grep keyboard

# Test /key topic
ros2 topic echo /key
# Then press keys - you should see output

# If nothing, keyboard node may have crashed:
ros2 run f1tenth_simulator keyboard
```

### Issue: Collision/TTC cuts off simulation

This is **intentional safety**. Behavior controller detects collision and halts all commands.

**To continue:**
```bash
# Use RViz to reset pose (see Part 3.4)
# Or use command line pose reset
ros2 topic pub /pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1, y: 1}, orientation: {w: 1.0}}}' --once

# Then re-enable control (press 'k' or 'r')
```

---

## Part 7: What's Next

1. **Learn ROS2 concepts**: Check [ROS2 docs](https://docs.ros.org/en/humble/)
2. **Implement your planner**: Subscribe to `/scan`, implement obstacle avoidance logic
3. **Tune parameters**: Adjust `scan_beams`, `max_speed`, dynamics parameters
4. **Test in sim first**: Use keyboard mode or random walk to verify physics before deploying to real car

---

## Reference: Key Files

- **Launch file**: `launch/simulator.launch.py` (Python, ROS2)
- **Parameters**: `params.yaml` (vehicle/sensor/control settings)
- **Map files**: `maps/*.yaml` (occupancy grids)
- **Source nodes**: `node/*.cpp` → Compiled to `install/lib/f1tenth_simulator/`
- **Headers/libraries**: `include/f1tenth_simulator/` + `src/` (physics/math)

---

## Version Notes

- **ROS**: Humble
- **Ubuntu**: 22.04
- **Build system**: colcon (ament_cmake)
- **C++ standard**: C++17
- **Status**: Migrated from ROS1 Melodic

For full migration details, see `ROS2_MIGRATION.md`.

