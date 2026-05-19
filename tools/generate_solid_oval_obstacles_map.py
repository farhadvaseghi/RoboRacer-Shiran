#!/usr/bin/env python3
"""Generate a solid-boundary oval map with two static circular obstacles."""

import os

from generate_solid_oval_map import (
    FREE_PIXEL,
    HEIGHT,
    OCCUPIED_PIXEL,
    RESOLUTION,
    WIDTH,
    is_drivable,
    pixel_to_world,
    write_pgm,
    write_yaml,
)


STATIC_OBSTACLES = (
    {'center': (19.5, 0.55), 'radius': 0.40},
    {'center': (40.5, 4.45), 'radius': 0.40},
)


def in_static_obstacle(wx: float, wy: float) -> bool:
    for obstacle in STATIC_OBSTACLES:
        cx, cy = obstacle['center']
        radius = obstacle['radius']
        dx = wx - cx
        dy = wy - cy
        if dx * dx + dy * dy <= radius * radius:
            return True
    return False


def main() -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.normpath(os.path.join(script_dir, '..', 'maps'))
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path = os.path.join(maps_dir, 'solid_oval_track_obstacles.pgm')
    yaml_path = os.path.join(maps_dir, 'solid_oval_track_obstacles.yaml')

    image = bytearray([OCCUPIED_PIXEL] * (WIDTH * HEIGHT))
    for row in range(HEIGHT):
        for col in range(WIDTH):
            wx, wy = pixel_to_world(col, row)
            if is_drivable(wx, wy) and not in_static_obstacle(wx, wy):
                image[row * WIDTH + col] = FREE_PIXEL

    write_pgm(pgm_path, image)
    write_yaml(yaml_path, 'solid_oval_track_obstacles.pgm')

    print(f'Resolution: {RESOLUTION:.3f} m/pixel')
    print(f'Static obstacles: {STATIC_OBSTACLES}')
    print(f'Written: {pgm_path}')
    print(f'Written: {yaml_path}')


if __name__ == '__main__':
    main()
