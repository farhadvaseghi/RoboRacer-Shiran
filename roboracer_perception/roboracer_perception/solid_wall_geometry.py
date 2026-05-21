"""Shared geometry helpers for solid-wall visualization."""

from __future__ import annotations

import math


# Geometry must match tools/generate_solid_oval_map.py.
BASE_INNER_R = 1.5
BASE_OUTER_R = 3.5
LEFT_CENTER = (0.0, 2.5)
RIGHT_CENTER = (20.0, 2.5)
STRAIGHT_X_MIN = 0.0
STRAIGHT_X_MAX = 20.0
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


def segment_points_on_arc(
    center_x: float,
    center_y: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    max_segment_length: float,
) -> list[tuple[float, float]]:
    """Discretize an arc into points such that adjacent points are short."""
    angle_span = end_angle - start_angle
    steps = max(2, int(math.ceil(abs(angle_span) * radius / max_segment_length)) + 1)
    return [
        (
            center_x + radius * math.cos(start_angle + angle_span * i / (steps - 1)),
            center_y + radius * math.sin(start_angle + angle_span * i / (steps - 1)),
        )
        for i in range(steps)
    ]


def build_wall_segments(arc_segment_length: float) -> list[tuple[float, float, float, float]]:
    """Return line segments approximating the solid oval wall centerlines."""
    segments = [
        (STRAIGHT_X_MIN, BOTTOM_Y_MIN, STRAIGHT_X_MAX, BOTTOM_Y_MIN),
        (STRAIGHT_X_MIN, BOTTOM_Y_MAX, STRAIGHT_X_MAX, BOTTOM_Y_MAX),
        (STRAIGHT_X_MIN, TOP_Y_MIN, STRAIGHT_X_MAX, TOP_Y_MIN),
        (STRAIGHT_X_MIN, TOP_Y_MAX, STRAIGHT_X_MAX, TOP_Y_MAX),
    ]

    left_inner = segment_points_on_arc(
        LEFT_CENTER[0], LEFT_CENTER[1], INNER_R, math.pi / 2.0, 3.0 * math.pi / 2.0,
        arc_segment_length,
    )
    left_outer = segment_points_on_arc(
        LEFT_CENTER[0], LEFT_CENTER[1], OUTER_R, math.pi / 2.0, 3.0 * math.pi / 2.0,
        arc_segment_length,
    )
    right_inner = segment_points_on_arc(
        RIGHT_CENTER[0], RIGHT_CENTER[1], INNER_R, -math.pi / 2.0, math.pi / 2.0,
        arc_segment_length,
    )
    right_outer = segment_points_on_arc(
        RIGHT_CENTER[0], RIGHT_CENTER[1], OUTER_R, -math.pi / 2.0, math.pi / 2.0,
        arc_segment_length,
    )

    for point_set in (left_inner, left_outer, right_inner, right_outer):
        for p1, p2 in zip(point_set[:-1], point_set[1:]):
            segments.append((p1[0], p1[1], p2[0], p2[1]))

    return segments
