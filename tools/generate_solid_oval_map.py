#!/usr/bin/env python3
"""Generate a solid-boundary oval occupancy-grid map for RoboRacer.

The geometry matches the existing oval track so the robot spawn pose and
vehicle scale remain valid, but the lane boundaries are continuous occupied
walls instead of discrete cone footprints.

Compared with the first solid-wall version, this variant adds extra clearance:
  - straights are widened, with more room on the inner side
  - turns get additional radial margin, especially on the inner pass
  - this gives the moving obstacle and ego more room in overtaking and
    static-obstacle scenarios without changing the overall oval layout
"""

import os


RESOLUTION = 0.025
ORIGIN_X = -10.0
ORIGIN_Y = -6.0
WIDTH = 2560
HEIGHT = 1040

FREE_PIXEL = 255
OCCUPIED_PIXEL = 0

BASE_INNER_R = 1.5
BASE_OUTER_R = 3.5
LEFT_CENTER = (0.0, 2.5)
RIGHT_CENTER = (20.0, 2.5)
STRAIGHT_X_MIN = 0.0
STRAIGHT_X_MAX = 20.0

# Additional room beyond the original 2.0 m lane.
OUTER_STRAIGHT_MARGIN = 0.35
INNER_STRAIGHT_MARGIN = 0.50
OUTER_CURVE_MARGIN = 0.35
INNER_CURVE_MARGIN = 0.55

INNER_R = BASE_INNER_R - INNER_CURVE_MARGIN
OUTER_R = BASE_OUTER_R + OUTER_CURVE_MARGIN
BOTTOM_Y_MIN = -1.0 - OUTER_STRAIGHT_MARGIN
BOTTOM_Y_MAX = 1.0 + INNER_STRAIGHT_MARGIN
TOP_Y_MIN = 4.0 - INNER_STRAIGHT_MARGIN
TOP_Y_MAX = 6.0 + OUTER_STRAIGHT_MARGIN

def pixel_to_world(col: int, row: int) -> tuple[float, float]:
    """Return the world coordinates at the center of a pixel."""
    wx = ORIGIN_X + (col + 0.5) * RESOLUTION
    wy = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
    return wx, wy


def in_bottom_straight(wx: float, wy: float) -> bool:
    return STRAIGHT_X_MIN <= wx <= STRAIGHT_X_MAX and BOTTOM_Y_MIN <= wy <= BOTTOM_Y_MAX


def in_top_straight(wx: float, wy: float) -> bool:
    return STRAIGHT_X_MIN <= wx <= STRAIGHT_X_MAX and TOP_Y_MIN <= wy <= TOP_Y_MAX


def in_left_turn(wx: float, wy: float) -> bool:
    dx = wx - LEFT_CENTER[0]
    dy = wy - LEFT_CENTER[1]
    radius_sq = dx * dx + dy * dy
    return wx <= LEFT_CENTER[0] and INNER_R * INNER_R <= radius_sq <= OUTER_R * OUTER_R


def in_right_turn(wx: float, wy: float) -> bool:
    dx = wx - RIGHT_CENTER[0]
    dy = wy - RIGHT_CENTER[1]
    radius_sq = dx * dx + dy * dy
    return wx >= RIGHT_CENTER[0] and INNER_R * INNER_R <= radius_sq <= OUTER_R * OUTER_R


def is_drivable(wx: float, wy: float) -> bool:
    return (
        in_bottom_straight(wx, wy)
        or in_top_straight(wx, wy)
        or in_left_turn(wx, wy)
        or in_right_turn(wx, wy)
    )


def write_pgm(file_path: str, image: bytearray) -> None:
    with open(file_path, 'wb') as f:
        header = f'P5\n{WIDTH} {HEIGHT}\n255\n'
        f.write(header.encode('ascii'))
        f.write(bytes(image))


def write_yaml(file_path: str, pgm_name: str) -> None:
    content = (
        f'image: {pgm_name}\n'
        f'resolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n'
    )
    with open(file_path, 'w', encoding='ascii') as f:
        f.write(content)


def main() -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.normpath(os.path.join(script_dir, '..', 'maps'))
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path = os.path.join(maps_dir, 'solid_oval_track.pgm')
    yaml_path = os.path.join(maps_dir, 'solid_oval_track.yaml')

    image = bytearray([OCCUPIED_PIXEL] * (WIDTH * HEIGHT))
    for row in range(HEIGHT):
        for col in range(WIDTH):
            wx, wy = pixel_to_world(col, row)
            if is_drivable(wx, wy):
                image[row * WIDTH + col] = FREE_PIXEL

    write_pgm(pgm_path, image)
    write_yaml(yaml_path, 'solid_oval_track.pgm')

    lane_width_straight = BOTTOM_Y_MAX - BOTTOM_Y_MIN
    lane_width_curve = OUTER_R - INNER_R
    print(
        'Solid oval dimensions: '
        f'straight width={lane_width_straight:.2f} m, '
        f'curve width={lane_width_curve:.2f} m'
    )
    print(f'Written: {pgm_path}')
    print(f'Written: {yaml_path}')


if __name__ == '__main__':
    main()
