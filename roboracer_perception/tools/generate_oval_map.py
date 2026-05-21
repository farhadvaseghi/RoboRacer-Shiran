#!/usr/bin/env python3
"""Generate an oval racetrack occupancy-grid map for use with RoboRacer.

Outputs:
    maps/oval_track.pgm  — P5 binary PGM, white=255 (free), black=0 (cone)
    maps/oval_track.yaml — ROS-compatible map metadata

Track geometry (all dimensions in metres):
    Oval centreline:
        Bottom straight: y=0,   x in [0, 20]
        Right turn:      semicircle, centre=(20, 2.5), radius=2.5
        Top straight:    y=5,   x in [20, 0]
        Left turn:       semicircle, centre=(0,  2.5), radius=2.5
    Total track width: 2 m → cones at ±1.0 m from centreline

Cone spacing:
    Straights: 1.5 m
    Curves:    every 15 degrees

Map canvas: x in [-3, 27], y in [-3, 10]  (30 m × 13 m)
Resolution:  0.025 m/pixel → 1200 × 520 pixels
Origin:      (-3.0, -3.0, 0.0)
"""

import math
import os
import struct


# ---------------------------------------------------------------------------
# Map configuration
# ---------------------------------------------------------------------------

RESOLUTION = 0.025        # metres per pixel
ORIGIN_X   = -5.0         # metres (world x at pixel col=0)
ORIGIN_Y   = -3.0         # metres (world y at pixel row=height-1)

WORLD_W = 32.0            # total world width  (x from -5 to 27)
WORLD_H = 13.0            # total world height (y from -3 to 10)

WIDTH  = int(WORLD_W / RESOLUTION)   # 1200 pixels
HEIGHT = int(WORLD_H / RESOLUTION)   # 520  pixels

FREE_PIXEL = 255
CONE_PIXEL = 0
CONE_RADIUS_PX = 5        # pixels ≈ 0.125 m physical radius


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
    """Convert world (x, y) in metres to (col, row) pixel coordinates."""
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int(HEIGHT - 1 - (wy - ORIGIN_Y) / RESOLUTION)
    return col, row


# ---------------------------------------------------------------------------
# Cone placement — returns list of (wx, wy) world positions
# ---------------------------------------------------------------------------

def straight_cones(x_start: float, x_end: float, y: float,
                   spacing: float) -> list[tuple[float, float]]:
    """Place cones along a straight at fixed spacing.

    Args:
        x_start: Starting x (inclusive first cone at x_start).
        x_end:   Ending x (cones up to but not exceeding x_end).
        y:       Fixed y coordinate.
        spacing: Distance between cones.

    Returns:
        List of (wx, wy) world positions.
    """
    cones = []
    x = x_start
    step = spacing if x_end >= x_start else -spacing
    while (step > 0 and x <= x_end) or (step < 0 and x >= x_end):
        cones.append((x, y))
        x += step
    return cones


def arc_cones(cx: float, cy: float, radius: float,
              angle_start_deg: float, angle_end_deg: float,
              step_deg: float) -> list[tuple[float, float]]:
    """Place cones around an arc.

    Angles are measured from the positive-x axis (standard maths convention).
    The arc goes from angle_start_deg to angle_end_deg inclusive.

    Args:
        cx, cy:          Arc centre (world metres).
        radius:          Arc radius (metres).
        angle_start_deg: Start angle (degrees).
        angle_end_deg:   End angle (degrees).
        step_deg:        Angular spacing (degrees), positive.

    Returns:
        List of (wx, wy) world positions.
    """
    cones = []
    a = angle_start_deg
    step = step_deg if angle_end_deg >= angle_start_deg else -step_deg
    # Normalise direction
    if angle_end_deg < angle_start_deg:
        step = -abs(step_deg)
    else:
        step = abs(step_deg)

    while (step > 0 and a <= angle_end_deg + 1e-6) or \
          (step < 0 and a >= angle_end_deg - 1e-6):
        rad = math.radians(a)
        wx = cx + radius * math.cos(rad)
        wy = cy + radius * math.sin(rad)
        cones.append((wx, wy))
        a += step
    return cones


def generate_all_cones() -> list[tuple[float, float]]:
    """Return the world positions of all cone centres."""
    cones = []

    spacing = 1.5
    step_deg = 15.0

    # ---- Straights --------------------------------------------------------

    # Bottom straight — inner boundary (left of car heading +x): y = +1.0
    cones += straight_cones(1.0, 19.5, 1.0, spacing)
    # Bottom straight — outer boundary (right of car heading +x): y = -1.0
    cones += straight_cones(1.0, 19.5, -1.0, spacing)

    # Top straight — inner boundary (left of car heading −x): y = 4.0
    cones += straight_cones(1.0, 19.5, 4.0, spacing)
    # Top straight — outer boundary (right of car heading −x): y = 6.0
    cones += straight_cones(1.0, 19.5, 6.0, spacing)

    # ---- Right turn: centre (20, 2.5), angles −80° to +80° ----------------
    # Inner arc: radius 1.5 m (closer to centre of oval)
    cones += arc_cones(20.0, 2.5, 1.5, -80.0, 80.0, step_deg)
    # Outer arc: radius 3.5 m
    cones += arc_cones(20.0, 2.5, 3.5, -80.0, 80.0, step_deg)

    # ---- Left turn: centre (0, 2.5), angles 100° to 260° ------------------
    # Inner arc: radius 1.5 m
    cones += arc_cones(0.0, 2.5, 1.5, 100.0, 260.0, step_deg)
    # Outer arc: radius 3.5 m
    cones += arc_cones(0.0, 2.5, 3.5, 100.0, 260.0, step_deg)

    return cones


# ---------------------------------------------------------------------------
# Image drawing
# ---------------------------------------------------------------------------

def create_image(width: int, height: int, fill: int) -> bytearray:
    """Create a flat bytearray representing a grayscale image."""
    return bytearray([fill] * (width * height))


def draw_cone(image: bytearray, width: int, height: int,
              col: int, row: int, radius_px: int, value: int) -> None:
    """Draw a filled circle (cone footprint) onto the flat image array."""
    for dr in range(-radius_px, radius_px + 1):
        for dc in range(-radius_px, radius_px + 1):
            if dc * dc + dr * dr <= radius_px * radius_px:
                r = row + dr
                c = col + dc
                if 0 <= r < height and 0 <= c < width:
                    image[r * width + c] = value


# ---------------------------------------------------------------------------
# File writers
# ---------------------------------------------------------------------------

def write_pgm(file_path: str, image: bytearray,
              width: int, height: int) -> None:
    """Write a P5 (binary) PGM file."""
    with open(file_path, 'wb') as f:
        # PGM header (ASCII)
        header = f'P5\n{width} {height}\n255\n'
        f.write(header.encode('ascii'))
        # Binary pixel data — one byte per pixel
        f.write(bytes(image))


def write_yaml(file_path: str, pgm_name: str) -> None:
    """Write the ROS map metadata YAML file."""
    content = (
        f'image: {pgm_name}\n'
        f'resolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n'
    )
    with open(file_path, 'w') as f:
        f.write(content)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    maps_dir = os.path.normpath(maps_dir)
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path  = os.path.join(maps_dir, 'oval_track.pgm')
    yaml_path = os.path.join(maps_dir, 'oval_track.yaml')

    print(f'Canvas: {WIDTH} × {HEIGHT} pixels '
          f'({WORLD_W} m × {WORLD_H} m at {RESOLUTION} m/px)')

    # Initialise all pixels to FREE (white)
    image = create_image(WIDTH, HEIGHT, FREE_PIXEL)

    # Generate cone positions and draw them
    cones = generate_all_cones()
    print(f'Placing {len(cones)} cones ...')

    skipped = 0
    for wx, wy in cones:
        col, row = world_to_pixel(wx, wy)
        if (0 <= col < WIDTH) and (0 <= row < HEIGHT):
            draw_cone(image, WIDTH, HEIGHT, col, row, CONE_RADIUS_PX, CONE_PIXEL)
        else:
            skipped += 1
            print(f'  WARNING: cone at world ({wx:.2f}, {wy:.2f}) → '
                  f'pixel ({col}, {row}) is outside canvas — skipped.')

    if skipped:
        print(f'{skipped} cones were outside the canvas and skipped.')

    # Write files
    write_pgm(pgm_path, image, WIDTH, HEIGHT)
    print(f'Written: {pgm_path}')

    write_yaml(yaml_path, 'oval_track.pgm')
    print(f'Written: {yaml_path}')

    print('Done.')


if __name__ == '__main__':
    main()
