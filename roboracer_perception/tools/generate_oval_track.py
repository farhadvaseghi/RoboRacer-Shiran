#!/usr/bin/env python3
"""Generate the oval race track Gazebo world, including random obstacles.

Track geometry
--------------
  Bottom straight : blue cones at y=+1.0, yellow at y=-1.0, x = 1..19 step 1.5
  Top straight    : blue cones at y=+4.0, yellow at y=+6.0, x = 1..19 step 1.5
  Left  turn      : center (0, 2.5),  blue r=1.5, yellow r=3.5, 100°..260°
  Right turn      : center (20, 2.5), blue r=1.5, yellow r=3.5, -80°..80°
  Start gate      : orange cones at x=0.5, y=±1.0

Random obstacles (seed=42) — barrels placed in the drivable lanes
  6 on bottom straight (center lane y≈0)
  6 on top straight    (center lane y≈5)

Output
------
  ../worlds/oval_track.world

Usage
-----
  cd ~/roboracer_ws/src/roboracer_perception/tools
  python3 generate_oval_track.py
"""

import math
import os

import numpy as np

# ── Output ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_PATH   = os.path.join(SCRIPT_DIR, '..', 'worlds', 'oval_track.world')

# ── Geometry ──────────────────────────────────────────────────────────────
STRAIGHT_X     = np.arange(1.0, 20.0, 1.5)   # x positions on straights
BLUE_Y_BOT     =  1.0    # blue (left) wall, bottom straight
YELLOW_Y_BOT   = -1.0    # yellow (right) wall, bottom straight
BLUE_Y_TOP     =  4.0
YELLOW_Y_TOP   =  6.0
LEFT_CENTER    = (0.0,  2.5)
RIGHT_CENTER   = (20.0, 2.5)
INNER_R        = 1.5
OUTER_R        = 3.5
ARC_STEP_DEG   = 10       # angular spacing of cones on arcs

# ── SDF helpers ───────────────────────────────────────────────────────────

def _cone_include(color: str, name: str, x: float, y: float) -> str:
    return (
        f'    <include>\n'
        f'      <uri>model://{color}_cone</uri>\n'
        f'      <name>{name}</name>\n'
        f'      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>\n'
        f'    </include>\n'
    )


def _obstacle(idx: int, x: float, y: float) -> str:
    """Inline barrel obstacle — red cylinder, radius 0.10 m, height 0.30 m."""
    return (
        f'    <model name="obstacle_{idx}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.4f} {y:.4f} 0.15 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="col">\n'
        f'          <geometry>'
        f'<cylinder><radius>0.10</radius><length>0.30</length></cylinder>'
        f'</geometry>\n'
        f'        </collision>\n'
        f'        <visual name="vis">\n'
        f'          <geometry>'
        f'<cylinder><radius>0.10</radius><length>0.30</length></cylinder>'
        f'</geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.75 0.20 0.10 1</ambient>\n'
        f'            <diffuse>0.75 0.20 0.10 1</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


# ── Cone placement ────────────────────────────────────────────────────────

def generate_cones():
    blue, yellow, orange = [], [], []
    idx_b = idx_y = 0

    # Bottom straight
    for x in STRAIGHT_X:
        blue.append((f'blue_cone_{idx_b}',   x, BLUE_Y_BOT))
        yellow.append((f'yellow_cone_{idx_y}', x, YELLOW_Y_BOT))
        idx_b += 1; idx_y += 1

    # Top straight
    for x in STRAIGHT_X:
        blue.append((f'blue_cone_{idx_b}',   x, BLUE_Y_TOP))
        yellow.append((f'yellow_cone_{idx_y}', x, YELLOW_Y_TOP))
        idx_b += 1; idx_y += 1

    # Left turn (center 0, 2.5) — arcs from 100° to 260°
    cx, cy = LEFT_CENTER
    for deg in range(100, 261, ARC_STEP_DEG):
        rad = math.radians(deg)
        blue.append((f'blue_cone_{idx_b}',
                     cx + INNER_R * math.cos(rad), cy + INNER_R * math.sin(rad)))
        yellow.append((f'yellow_cone_{idx_y}',
                       cx + OUTER_R * math.cos(rad), cy + OUTER_R * math.sin(rad)))
        idx_b += 1; idx_y += 1

    # Right turn (center 20, 2.5) — arcs from -80° to 80°
    cx, cy = RIGHT_CENTER
    for deg in range(-80, 81, ARC_STEP_DEG):
        rad = math.radians(deg)
        blue.append((f'blue_cone_{idx_b}',
                     cx + INNER_R * math.cos(rad), cy + INNER_R * math.sin(rad)))
        yellow.append((f'yellow_cone_{idx_y}',
                       cx + OUTER_R * math.cos(rad), cy + OUTER_R * math.sin(rad)))
        idx_b += 1; idx_y += 1

    # Start gate
    orange = [
        ('orange_cone_0', 0.5,  1.0),
        ('orange_cone_1', 0.5, -1.0),
    ]

    return blue, yellow, orange


# ── Obstacle placement ────────────────────────────────────────────────────

def generate_obstacles(seed: int = 42, n_per_straight: int = 6):
    rng = np.random.default_rng(seed)
    obstacles = []

    # Bottom straight: drivable lane y ∈ (-0.85, 0.85), x ∈ (2.5, 18.0)
    xs_bot = rng.uniform(2.5, 18.0, n_per_straight)
    ys_bot = rng.uniform(-0.60, 0.60, n_per_straight)
    for i, (x, y) in enumerate(zip(xs_bot, ys_bot)):
        obstacles.append((i, x, y))

    # Top straight: drivable lane y ∈ (4.15, 5.85), x ∈ (2.5, 18.0)
    xs_top = rng.uniform(2.5, 18.0, n_per_straight)
    ys_top = rng.uniform(4.15, 5.85, n_per_straight)
    for i, (x, y) in enumerate(zip(xs_top, ys_top), start=n_per_straight):
        obstacles.append((i, x, y))

    return obstacles


# ── World builder ─────────────────────────────────────────────────────────

def build_world(blue, yellow, orange, obstacles) -> str:
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        '  <world name="oval_track">',
        '',
        '    <!-- Lighting -->',
        '    <light name="sun" type="directional">',
        '      <cast_shadows>true</cast_shadows>',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.9 0.9 0.9 1</diffuse>',
        '      <specular>0.2 0.2 0.2 1</specular>',
        '      <direction>-0.5 0.1 -0.9</direction>',
        '    </light>',
        '',
        '    <!-- Physics -->',
        '    <physics type="ode">',
        '      <real_time_update_rate>1000</real_time_update_rate>',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1</real_time_factor>',
        '    </physics>',
        '',
        '    <!-- Ground plane -->',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <link name="link">',
        '        <collision name="collision">',
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>100 100</size></plane></geometry>',
        '          <surface><friction><ode>'
        '<mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>100 100</size></plane></geometry>',
        '          <material>',
        '            <ambient>0.4 0.4 0.4 1</ambient>',
        '            <diffuse>0.4 0.4 0.4 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Track surface (dark tarmac) -->',
        '    <model name="track_surface">',
        '      <static>true</static>',
        '      <pose>10 2.5 0.001 0 0 0</pose>',
        '      <link name="link">',
        '        <visual name="visual">',
        '          <geometry><box size="28 12 0.001"/></geometry>',
        '          <material>',
        '            <ambient>0.15 0.15 0.15 1</ambient>',
        '            <diffuse>0.15 0.15 0.15 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Blue cones (left boundary) -->',
    ]

    for name, x, y in blue:
        lines.append(_cone_include('blue', name, x, y))

    lines.append('')
    lines.append('    <!-- Yellow cones (right boundary) -->')
    for name, x, y in yellow:
        lines.append(_cone_include('yellow', name, x, y))

    lines.append('')
    lines.append('    <!-- Orange cones (start/finish gate) -->')
    for name, x, y in orange:
        lines.append(_cone_include('orange', name, x, y))

    lines.append('')
    lines.append('    <!-- Random obstacles (barrels) -->')
    for idx, x, y in obstacles:
        lines.append(_obstacle(idx, x, y))

    lines.append('')
    lines.append('  </world>')
    lines.append('</sdf>')
    return '\n'.join(lines)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    blue, yellow, orange = generate_cones()
    obstacles = generate_obstacles(seed=42, n_per_straight=6)

    print(f'Blue cones    : {len(blue)}')
    print(f'Yellow cones  : {len(yellow)}')
    print(f'Orange cones  : {len(orange)}')
    print(f'Obstacles     : {len(obstacles)}')

    world_sdf = build_world(blue, yellow, orange, obstacles)

    out = os.path.normpath(OUT_PATH)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(world_sdf)
    print(f'Written → {out}')


if __name__ == '__main__':
    main()
