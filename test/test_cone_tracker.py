"""Unit tests for roboracer_perception.cone_tracker."""

import math

from roboracer_perception.msg import Cone
from roboracer_perception.cone_tracker import (
    Track,
    associate_tracks,
    body_to_world,
    confirmed_tracks,
    transform_detections_to_world,
    update_tracks,
    yaw_from_quaternion,
)


class TestGeometryHelpers:

    def test_yaw_from_identity_quaternion_is_zero(self):
        assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0

    def test_body_to_world_applies_rotation_and_translation(self):
        x_world, y_world = body_to_world(
            x_body=1.0,
            y_body=0.0,
            pose_x=2.0,
            pose_y=3.0,
            yaw=math.pi / 2.0,
        )
        assert math.isclose(x_world, 2.0, abs_tol=1e-6)
        assert math.isclose(y_world, 4.0, abs_tol=1e-6)

    def test_transform_detections_to_world_updates_each_detection(self):
        detections = [{
            'x': 2.0,
            'y': -1.0,
            'color': Cone.COLOR_UNKNOWN,
            'confidence': 0.8,
            'radius': 0.06,
        }]
        world = transform_detections_to_world(
            detections, pose_x=10.0, pose_y=5.0, yaw=0.0)
        assert len(world) == 1
        assert world[0]['x'] == 12.0
        assert world[0]['y'] == 4.0


class TestAssociation:

    def test_associate_tracks_matches_nearest_pairs_once(self):
        tracks = [
            Track(x=0.0, y=0.0, color=Cone.COLOR_UNKNOWN, confidence=0.5, radius=0.05),
            Track(x=5.0, y=0.0, color=Cone.COLOR_UNKNOWN, confidence=0.5, radius=0.05),
        ]
        detections = [
            {'x': 0.1, 'y': 0.0, 'color': Cone.COLOR_BLUE, 'confidence': 0.9, 'radius': 0.06},
            {'x': 5.2, 'y': 0.0, 'color': Cone.COLOR_YELLOW, 'confidence': 0.9, 'radius': 0.06},
        ]

        matches, unmatched_tracks, unmatched_detections = associate_tracks(
            tracks, detections, max_distance=0.5)

        assert matches == [(0, 0), (1, 1)]
        assert unmatched_tracks == []
        assert unmatched_detections == []


class TestTrackUpdate:

    def test_update_tracks_creates_and_confirms_track_after_repeated_hits(self):
        tracks = []
        detections = [{
            'x': 1.0,
            'y': 2.0,
            'color': Cone.COLOR_BLUE,
            'confidence': 0.9,
            'radius': 0.06,
        }]

        for _ in range(3):
            tracks = update_tracks(
                tracks,
                detections,
                association_distance=0.4,
                position_alpha=0.2,
                max_missed_frames=2,
            )

        confirmed = confirmed_tracks(tracks, min_observations=3)
        assert len(confirmed) == 1
        assert confirmed[0].color == Cone.COLOR_BLUE
        assert confirmed[0].hits == 3

    def test_update_tracks_preserves_known_color_when_detection_is_unknown(self):
        tracks = [Track(
            x=1.0,
            y=1.0,
            color=Cone.COLOR_YELLOW,
            confidence=0.5,
            radius=0.06,
        )]
        detections = [{
            'x': 1.05,
            'y': 1.02,
            'color': Cone.COLOR_UNKNOWN,
            'confidence': 0.8,
            'radius': 0.05,
        }]

        tracks = update_tracks(
            tracks,
            detections,
            association_distance=0.2,
            position_alpha=0.5,
            max_missed_frames=2,
        )

        assert len(tracks) == 1
        assert tracks[0].color == Cone.COLOR_YELLOW

    def test_update_tracks_prunes_stale_tracks(self):
        tracks = [Track(
            x=3.0,
            y=0.0,
            color=Cone.COLOR_UNKNOWN,
            confidence=0.4,
            radius=0.05,
        )]

        tracks = update_tracks(
            tracks,
            detections=[],
            association_distance=0.5,
            position_alpha=0.2,
            max_missed_frames=0,
        )

        assert tracks == []
