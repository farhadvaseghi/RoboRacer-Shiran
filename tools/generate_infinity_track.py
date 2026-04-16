#!/usr/bin/env python3
"""Generate an infinity-symbol (lemniscate of Bernoulli) Gazebo world.

The track center line is the lemniscate of Bernoulli:
    x(t) = A * sqrt(2) * cos(t)       / (1 + sin²(t))
    y(t) = A * sqrt(2) * cos(t)*sin(t) / (1 + sin²(t))

Blue cones  → left  boundary (from driver's perspective)
Yellow cones→ right boundary
Orange cones→ start/finish gate (rightmost tip of the figure-8)

Output
------
  ../worlds/infinity_track.world   — SDF world file for Gazebo Classic

Usage
-----
  cd ~/roboracer_ws/src/roboracer_perception/tools
  python3 generate_infinity_track.py
"""

import math
import os

import numpy as np

# ── Track geometry ─────────────────────────────────────────────────────────
A            = 7.0     # lemniscate scale (m); rightmost point x ≈ A*√2 ≈ 9.9 m
HALF_WIDTH   = 0.80    # half track width (m); total drivable width = 1.6 m
CONE_SPACING = 1.20    # arc-length between cones (m)
N_SAMPLE     = 4000    # internal resolution for curve integration

# ── Output path ────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
OUT_PATH     = os.path.join(SCRIPT_DIR, '..', 'worlds', 'infinity_track.world')

# ── Helper: lemniscate of Bernoulli ────────────────────────────────────────

def lemniscate(t: np.ndarray, a: float):
    """Return (x, y) arrays for the lemniscate of Bernoulli."""
    s2 = np.sin(t) ** 2
    denom = 1.0 + s2
    x = a * math.sqrt(2) * np.cos(t) / denom
    y = a * math.sqrt(2) * np.cos(t) * np.sin(t) / denom
    return x, y


def resample_curve(cx, cy, spacing):
    """Resample (cx, cy) at equal arc-length intervals of `spacing`."""
    dx = np.diff(cx, append=cx[0])
    dy = np.diff(cy, append=cy[0])
    ds = np.sqrt(dx ** 2 + dy ** 2)
    s  = np.concatenate([[0.0], np.cumsum(ds[:-1])])
    total = s[-1]

    n = max(2, int(total / spacing))
    s_uniform = np.linspace(0.0, total, n, endpoint=False)
    return (
        np.interp(s_uniform, s, cx),
        np.interp(s_uniform, s, cy),
        s_uniform,
        s,
        total,
    )


def compute_normals(cx_u, cy_u):
    """Unit left-normals (perpendicular to forward direction) at each point."""
    # Forward tangent via central differences on the resampled curve
    dx = np.gradient(cx_u)
    dy = np.gradient(cy_u)
    mag = np.maximum(np.sqrt(dx ** 2 + dy ** 2), 1e-9)
    tx = dx / mag
    ty = dy / mag
    # Left normal = 90° CCW from tangent
    return -ty, tx


# ── SDF generation ─────────────────────────────────────────────────────────

def _include(name: str, x: float, y: float, idx: int) -> str:
    return (
        f'    <include>\n'
        f'      <name>{name}_{idx:03d}</name>\n'
        f'      <uri>model://{name}</uri>\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>\n'
        f'    </include>\n'
    )


def generate_obstacles(cx_u, cy_u, seed: int = 42, n: int = 6):
    """Place n obstacles in each lobe by sampling points near the center line."""
    rng = np.random.default_rng(seed)
    obstacles = []

    # Right lobe: points where cx_u > 1.5 m
    right_mask = cx_u > 1.5
    right_idx  = np.where(right_mask)[0]
    chosen_r   = rng.choice(right_idx, size=min(n, len(right_idx)), replace=False)
    for i, idx in enumerate(chosen_r):
        jitter_x = rng.uniform(-0.25, 0.25)
        jitter_y = rng.uniform(-0.25, 0.25)
        obstacles.append((i, float(cx_u[idx]) + jitter_x,
                             float(cy_u[idx]) + jitter_y))

    # Left lobe: points where cx_u < -1.5 m
    left_mask = cx_u < -1.5
    left_idx  = np.where(left_mask)[0]
    chosen_l  = rng.choice(left_idx, size=min(n, len(left_idx)), replace=False)
    for i, idx in enumerate(chosen_l, start=n):
        jitter_x = rng.uniform(-0.25, 0.25)
        jitter_y = rng.uniform(-0.25, 0.25)
        obstacles.append((i, float(cx_u[idx]) + jitter_x,
                             float(cy_u[idx]) + jitter_y))

    return obstacles


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


def build_world(blue_cones, yellow_cones, orange_cones, obstacles=None) -> str:
    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.6">',
        '  <world name="infinity_track">',
        '',
        '    <!-- Lighting -->',
        '    <light name="sun" type="directional">',
        '      <cast_shadows>true</cast_shadows>',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.9 0.9 0.9 1</diffuse>',
        '      <specular>0.3 0.3 0.3 1</specular>',
        '      <direction>-0.5 0.2 -0.9</direction>',
        '    </light>',
        '',
        '    <!-- Ground (dark tarmac) -->',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <link name="link">',
        '        <collision name="collision">',
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>60 30</size></plane></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>60 30</size></plane></geometry>',
        '          <material>',
        '            <ambient>0.15 0.15 0.15 1</ambient>',
        '            <diffuse>0.15 0.15 0.15 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Physics -->',
        '    <physics type="ode">',
        '      <real_time_update_rate>1000</real_time_update_rate>',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1</real_time_factor>',
        '    </physics>',
        '',
        '    <!-- Blue cones (left / inner boundary) -->',
    ]

    for i, (x, y) in enumerate(blue_cones):
        lines.append(_include('blue_cone', x, y, i))

    lines.append('')
    lines.append('    <!-- Yellow cones (right / outer boundary) -->')
    for i, (x, y) in enumerate(yellow_cones):
        lines.append(_include('yellow_cone', x, y, i))

    lines.append('')
    lines.append('    <!-- Orange cones (start / finish gate) -->')
    for i, (x, y) in enumerate(orange_cones):
        lines.append(_include('orange_cone', x, y, i))

    if obstacles:
        lines.append('')
        lines.append('    <!-- Random obstacles (barrels) -->')
        for idx, x, y in obstacles:
            lines.append(_obstacle(idx, x, y))

    lines.append('')
    lines.append('  </world>')
    lines.append('</sdf>')
    return '\n'.join(lines)


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    # Sample center line at high resolution
    t = np.linspace(0, 2 * math.pi, N_SAMPLE, endpoint=False)
    cx, cy = lemniscate(t, A)

    # Resample at equal arc-length spacing
    cx_u, cy_u, _, s, total = resample_curve(cx, cy, CONE_SPACING)
    nx_u, ny_u = compute_normals(cx_u, cy_u)

    # Boundary cone positions
    blue_cones   = list(zip(
        cx_u + HALF_WIDTH * nx_u,
        cy_u + HALF_WIDTH * ny_u,
    ))
    yellow_cones = list(zip(
        cx_u - HALF_WIDTH * nx_u,
        cy_u - HALF_WIDTH * ny_u,
    ))

    # Orange gate at the rightmost tip (t≈0, x = A*√2, y = 0)
    tip_x = A * math.sqrt(2)
    orange_cones = [
        (tip_x + 0.2,  HALF_WIDTH + 0.3),   # left post
        (tip_x + 0.2, -(HALF_WIDTH + 0.3)), # right post
    ]

    obstacles = generate_obstacles(cx_u, cy_u, seed=42, n=6)

    print(f'Lemniscate total arc length : {total:.2f} m')
    print(f'Blue cones                  : {len(blue_cones)}')
    print(f'Yellow cones                : {len(yellow_cones)}')
    print(f'Orange cones                : {len(orange_cones)}')
    print(f'Obstacles                   : {len(obstacles)}')

    world_sdf = build_world(blue_cones, yellow_cones, orange_cones, obstacles)

    out = os.path.normpath(OUT_PATH)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(world_sdf)
    print(f'Written → {out}')


if __name__ == '__main__':
    main()
