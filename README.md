# Pure Pursuit Control – Path Tracking Module

This branch contains the implementation of the **Control module** for the RoboRacer project.

The controller is responsible for tracking the planned trajectory and generating steering and speed commands for the vehicle.

---

## How to Run

### Terminal 1 — Simulator

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator simulator.launch.py
```

---

### Terminal 2 — Navigation

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch f1tenth_simulator navigation.launch.py
```

---

### Terminal 3 — RViz

```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

- Use **Nav2 Goal** to send a target
- Nav2 generates a path on `/plan`
- Controller tracks the path

---

### Activate Navigation

```bash
ros2 topic pub --once /key std_msgs/msg/String "data: 'n'"
```

---

## System Architecture

```
Nav2 planner → /plan → Pure Pursuit Controller → /nav → mux → /drive → simulator
                              ↑
                            /odom
```

---

## Controller Overview

The control module is implemented as a standalone **Pure Pursuit controller** combined with a **PID speed controller**.

### Inputs

| Topic | Description |
|------|-------------|
| `/plan` | Global path from planner |
| `/odom` | Vehicle pose and velocity |

### Output

| Topic | Description |
|------|-------------|
| `/nav` | Ackermann control command |

---

## Control Logic

### Lateral Control (Pure Pursuit)

- selects a lookahead point on the path  
- computes steering angle based on geometry  
- supports forward and reverse motion  

---

### Longitudinal Control (PID)

- tracks a reference speed  
- smooth acceleration and deceleration  
- reduces speed in sharp turns  

---

## Tracking Errors

The controller computes:

- **Cross-track error (CTE)** → lateral distance from path  
- **Heading error** → orientation difference  

The closest point on the path is computed using **projection onto path segments**, ensuring accurate error measurement.

---

## RViz Debug

The controller publishes markers for visualization:

- target point  
- lookahead circle  
- steering direction (green arrow)  
- error vector (robot → closest path point)  

---

## Reverse Motion

The target point is transformed to the robot frame:

- `x_local > 0` → forward  
- `x_local < 0` → reverse  

---

## Improvements

- reduced lookahead distance  
- speed reduction in sharp turns  
- smoother steering behavior  

---

## Debug Topics

```bash
ros2 topic echo /plan
ros2 topic echo /odom
ros2 topic echo /nav
```

---

## Summary

This module provides:

- Pure Pursuit path tracking  
- PID-based speed control  
- accurate tracking error computation  
- RViz visualization for debugging  

It represents the **Control layer** of the autonomous driving stack.
