#!/usr/bin/env python3
"""Offline: check a candidate route against the served map before driving it.

Two independent things go wrong with hand-picked goals, and this reports both.

1. PLACEMENT -- is the pose in free space, and how far is the nearest wall?
   Anything at or inside robot_radius cannot be planned to at all ('Starting
   point in lethal space' / goal in lethal space).

2. HEADING FEASIBILITY -- every pose in a NavigateThroughPoses route is a hard
   SE(2) constraint: the incoming segment must ARRIVE on that heading and the
   next must DEPART on it. Where the demanded heading disagrees with the
   direction of travel, the planner buys the difference with a swing out and
   back, which is the hook seen in Foxglove on 2026-08-03. This prints the
   mismatch at every waypoint so an unusable route is caught on the desk.

Reads the map only. Publishes nothing, touches no running node.
"""

import math
import os
import sys

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

MAP_YAML = os.path.expanduser('~/rr_maps/corridor_despeck.yaml')

# nav2_params_real.yaml global_costmap
ROBOT_RADIUS = 0.22
INFLATION_RADIUS = 0.25

# Route start: the mission seeds this on /initialpose while arming.
START = ('start', 0.0, 0.0, 0.0)


def load_map(path):
    with open(path) as fh:
        meta = yaml.safe_load(fh)
    pgm = os.path.join(os.path.dirname(path), meta['image'])

    with open(pgm, 'rb') as fh:
        assert fh.readline().strip() == b'P5', 'expected a binary PGM'
        dims = fh.readline()
        while dims.startswith(b'#'):
            dims = fh.readline()
        width, height = (int(v) for v in dims.split())
        maxval = int(fh.readline())
        data = np.frombuffer(fh.read(width * height), dtype=np.uint8)

    img = data.reshape((height, width))
    occ = (maxval - img.astype(float)) / maxval          # 0 free .. 1 occupied
    if meta.get('negate', 0):
        occ = 1.0 - occ
    return meta, occ


def main():
    meta, occ = load_map(MAP_YAML)
    res = meta['resolution']
    ox, oy, _ = meta['origin']
    height, width = occ.shape

    occupied = occ >= meta['occupied_thresh']
    # EDT gives, for every free cell, the distance to the nearest occupied one.
    clearance = distance_transform_edt(~occupied) * res

    print('map %s  %dx%d @%.3f m  origin [%.2f, %.2f]'
          % (meta['image'], width, height, res, ox, oy))
    print('robot_radius %.2f m, inflation_radius %.2f m\n'
          % (ROBOT_RADIUS, INFLATION_RADIUS))

    goals = []
    for arg in sys.argv[1:]:
        name, x, y, z, w = arg.split(',')
        goals.append((name, float(x), float(y),
                      math.degrees(2.0 * math.atan2(float(z), float(w)))))
    if not goals:
        print('usage: rr_goal_check.py name,x,y,z,w [...]')
        return 1

    print('%-8s %8s %8s %9s %10s  %s'
          % ('name', 'x', 'y', 'yaw_deg', 'clear_m', 'placement'))
    for name, x, y, yaw in goals:
        col = int((x - ox) / res)
        row = height - 1 - int((y - oy) / res)
        if not (0 <= col < width and 0 <= row < height):
            print('%-8s %8.3f %8.3f %9.2f %10s  OFF THE MAP'
                  % (name, x, y, yaw, '-'))
            continue
        d = clearance[row, col]
        if occupied[row, col]:
            verdict = 'IN A WALL'
        elif d < ROBOT_RADIUS:
            verdict = 'LETHAL (inside robot_radius)'
        elif d < INFLATION_RADIUS:
            verdict = 'inside inflation - planner will resist'
        else:
            verdict = 'free'
        print('%-8s %8.3f %8.3f %9.2f %10.2f  %s'
              % (name, x, y, yaw, d, verdict))

    # Heading feasibility across the sequence, starting from the seeded start.
    seq = [START[:1] + START[1:3] + (START[3],)] + goals
    print('\nheading feasibility (route order as given):')
    print('%-18s %10s %10s %10s  %s'
          % ('leg', 'bearing', 'demanded', 'mismatch', 'note'))
    for i in range(len(seq) - 1):
        n0, x0, y0, yaw0 = seq[i]
        n1, x1, y1, yaw1 = seq[i + 1]
        bearing = math.degrees(math.atan2(y1 - y0, x1 - x0))
        arrive = (yaw1 - bearing + 180.0) % 360.0 - 180.0
        depart = (bearing - yaw0 + 180.0) % 360.0 - 180.0
        worst = max(abs(arrive), abs(depart))
        note = ('ok' if worst < 35 else
                'tight' if worst < 70 else
                'INFEASIBLE - expect a hook/loop')
        print('%-18s %10.1f %10.1f %10.1f  %s'
              % ('%s -> %s' % (n0, n1), bearing, yaw1, arrive, note))
        if abs(depart) > 35:
            print('%-18s %10s %10s %10.1f  departure from %s disagrees too'
                  % ('', '', '', depart, n0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
