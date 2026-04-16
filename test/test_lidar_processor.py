"""Unit tests for roboracer_perception.lidar_processor.

All tests are pure Python — no ROS runtime required.  The module's
processing logic is deliberately free of ROS dependencies so it can be
tested here with plain numpy arrays.

Synthetic scan geometry
-----------------------
The LiDAR sits at the origin facing the +x direction (ROS laser frame).
A cone at (cx, cy) with radius r is modelled as a vertical cylinder.
For a ray at bearing θ the perpendicular distance from the origin to the
ray is  d = |cx·sin θ − cy·cos θ|.  When d < r the ray intersects the
near surface of the cylinder at range:

    t = cx·cos θ + cy·sin θ  −  sqrt(r² − d²)

All cones are placed in the forward half-plane (x > 0) and far enough
apart that DBSCAN (eps = 0.12 m) never merges two different cones.
"""

import math

import numpy as np
import pytest

from roboracer_perception.lidar_processor import (
    circle_fit_confidence,
    cluster_and_fit,
    filter_scan,
    fit_circle,
)


# ---------------------------------------------------------------------------
# Shared scan geometry (matches UST-10LX and f1tenth_gym_ros sim.yaml)
# ---------------------------------------------------------------------------

NUM_BEAMS = 1081
FOV_RAD = 4.7124                            # 270°
ANGLE_MIN = -FOV_RAD / 2                    # −135°
ANGLE_INCREMENT = FOV_RAD / (NUM_BEAMS - 1) # ≈ 0.004363 rad / beam  (0.25°)

RANGE_MIN = 0.06
RANGE_MAX = 8.0
DBSCAN_EPS = 0.12
DBSCAN_MIN_SAMPLES = 3
CONE_R_MIN = 0.02
CONE_R_MAX = 0.15

POSITION_TOLERANCE_M = 0.05   # 50 mm — the acceptance criterion


# ---------------------------------------------------------------------------
# Synthetic scan helpers
# ---------------------------------------------------------------------------

def _cylinder_hits(cx: float, cy: float, radius: float) -> dict:
    """Return {beam_index: range} for every beam that strikes the cylinder."""
    hits = {}
    for i in range(NUM_BEAMS):
        theta = ANGLE_MIN + i * ANGLE_INCREMENT
        d = abs(cx * math.sin(theta) - cy * math.cos(theta))
        if d >= radius:
            continue
        t = cx * math.cos(theta) + cy * math.sin(theta) - math.sqrt(radius ** 2 - d ** 2)
        if t > 0:
            hits[i] = t
    return hits


def _make_ranges(*cones) -> list:
    """Build a full 1081-beam range array for one or more (cx, cy, radius) cones.

    Non-hitting beams are set to float('inf') so filter_scan drops them.
    When two cones overlap on the same beam the closer surface wins.
    """
    ranges = [float('inf')] * NUM_BEAMS
    for cx, cy, radius in cones:
        for i, r in _cylinder_hits(cx, cy, radius).items():
            if r < ranges[i]:
                ranges[i] = r
    return ranges


def _run_pipeline(ranges: list) -> list:
    """filter_scan → cluster_and_fit with default parameters."""
    points = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
    return cluster_and_fit(points, DBSCAN_EPS, DBSCAN_MIN_SAMPLES, CONE_R_MIN, CONE_R_MAX)


def _match_detections(detected: list, ground_truth: list, tol: float) -> list:
    """Greedy nearest-neighbour matching of detections to ground truth positions.

    Returns a list of (detection, gt) pairs for every ground-truth cone that
    has a detection within *tol* metres.  Each detection is used at most once.
    """
    remaining = list(detected)
    matches = []
    for gx, gy in ground_truth:
        best_idx, best_dist = None, float('inf')
        for idx, cone in enumerate(remaining):
            dist = math.hypot(cone['cx'] - gx, cone['cy'] - gy)
            if dist < best_dist:
                best_dist, best_idx = dist, idx
        if best_idx is not None and best_dist <= tol:
            matches.append((remaining.pop(best_idx), (gx, gy)))
    return matches


# ---------------------------------------------------------------------------
# filter_scan
# ---------------------------------------------------------------------------

class TestFilterScan:

    def test_empty_ranges_returns_empty_array(self):
        pts = filter_scan([], ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert pts.shape == (0, 2)

    def test_all_inf_returns_empty_array(self):
        ranges = [float('inf')] * 10
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert pts.shape == (0, 2)

    def test_below_range_min_is_dropped(self):
        # Single beam straight ahead (angle = 0) at 0.05 m — below 0.06 m limit
        ranges = [float('inf')] * NUM_BEAMS
        centre_beam = NUM_BEAMS // 2
        ranges[centre_beam] = 0.05
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == 0

    def test_above_range_max_is_dropped(self):
        ranges = [float('inf')] * NUM_BEAMS
        centre_beam = NUM_BEAMS // 2
        ranges[centre_beam] = 9.0
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == 0

    def test_valid_range_is_kept(self):
        ranges = [float('inf')] * NUM_BEAMS
        centre_beam = NUM_BEAMS // 2
        ranges[centre_beam] = 2.0
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == 1

    def test_centre_beam_projects_to_positive_x(self):
        """The centre beam of a symmetric 270° scan points straight ahead (+x)."""
        ranges = [float('inf')] * NUM_BEAMS
        centre_beam = NUM_BEAMS // 2
        r = 3.0
        ranges[centre_beam] = r
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == 1
        x, y = pts[0]
        assert abs(x - r) < 1e-6
        assert abs(y) < 1e-6

    def test_nan_range_is_dropped(self):
        ranges = [float('nan')] * 5
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == 0

    def test_point_count_matches_valid_beams(self):
        ranges = [float('inf')] * NUM_BEAMS
        valid_indices = [100, 300, 540, 700, 900]
        for i in valid_indices:
            ranges[i] = 1.5
        pts = filter_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
        assert len(pts) == len(valid_indices)


# ---------------------------------------------------------------------------
# fit_circle
# ---------------------------------------------------------------------------

class TestFitCircle:

    def test_perfect_circle_recovered(self):
        """Points sampled exactly on a circle should give back its parameters."""
        cx_true, cy_true, r_true = 2.0, 0.5, 0.10
        angles = np.linspace(0, math.pi, 20)          # front-facing arc
        pts = np.column_stack([
            cx_true + r_true * np.cos(angles),
            cy_true + r_true * np.sin(angles),
        ])
        result = fit_circle(pts)
        assert result is not None
        cx, cy, r = result
        assert abs(cx - cx_true) < 1e-4
        assert abs(cy - cy_true) < 1e-4
        assert abs(r - r_true) < 1e-4

    def test_noisy_circle_within_tolerance(self):
        """Gaussian noise ≤ 5 mm should not push the centre error above 10 mm."""
        rng = np.random.default_rng(42)
        cx_true, cy_true, r_true = 1.5, -0.3, 0.11
        angles = np.linspace(-math.pi / 3, math.pi / 3, 15)
        pts = np.column_stack([
            cx_true + r_true * np.cos(angles) + rng.normal(0, 0.005, 15),
            cy_true + r_true * np.sin(angles) + rng.normal(0, 0.005, 15),
        ])
        result = fit_circle(pts)
        assert result is not None
        cx, cy, _ = result
        assert math.hypot(cx - cx_true, cy - cy_true) < 0.010

    def test_two_points_returns_none(self):
        pts = np.array([[1.0, 0.0], [1.1, 0.0]])
        assert fit_circle(pts) is None

    def test_collinear_points_returns_none_or_huge_radius(self):
        """Collinear points are degenerate — either None or radius >> cone limit."""
        pts = np.array([[1.0, 0.0], [1.5, 0.0], [2.0, 0.0]])
        result = fit_circle(pts)
        # Acceptable: either None (degenerate) or an enormous radius that would
        # be rejected by the cone_radius_max validation downstream.
        if result is not None:
            _, _, r = result
            assert r > CONE_R_MAX

    def test_radius_is_positive(self):
        angles = np.linspace(0, math.pi, 10)
        pts = np.column_stack([2.0 + 0.1 * np.cos(angles), 0.1 * np.sin(angles)])
        result = fit_circle(pts)
        assert result is not None
        _, _, r = result
        assert r > 0


# ---------------------------------------------------------------------------
# circle_fit_confidence
# ---------------------------------------------------------------------------

class TestCircleFitConfidence:

    def test_perfect_fit_gives_score_one(self):
        cx, cy, r = 2.0, 0.0, 0.10
        angles = np.linspace(0, math.pi, 12)
        pts = np.column_stack([cx + r * np.cos(angles), cy + r * np.sin(angles)])
        score = circle_fit_confidence(pts, cx, cy, r)
        assert abs(score - 1.0) < 1e-6

    def test_large_residual_gives_low_score(self):
        # Points far from the fitted circle should yield near-zero confidence
        pts = np.array([[1.0, 0.0], [1.5, 0.5], [2.0, 0.0], [1.5, -0.5]])
        score = circle_fit_confidence(pts, cx=5.0, cy=5.0, radius=0.10)
        assert score == 0.0

    def test_score_bounded_between_zero_and_one(self):
        rng = np.random.default_rng(7)
        pts = rng.uniform(-3, 3, (20, 2))
        score = circle_fit_confidence(pts, cx=0.0, cy=0.0, radius=1.0)
        assert 0.0 <= score <= 1.0


# ---------------------------------------------------------------------------
# cluster_and_fit — unit-level edge cases
# ---------------------------------------------------------------------------

class TestClusterAndFit:

    def test_empty_input_returns_empty_list(self):
        pts = np.empty((0, 2))
        result = cluster_and_fit(pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES,
                                 CONE_R_MIN, CONE_R_MAX)
        assert result == []

    def test_fewer_than_min_samples_returns_empty(self):
        pts = np.array([[1.0, 0.0], [1.05, 0.0]])   # only 2 points
        result = cluster_and_fit(pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES,
                                 CONE_R_MIN, CONE_R_MAX)
        assert result == []

    def test_oversized_cluster_is_rejected(self):
        """A cluster whose fitted radius exceeds cone_radius_max is discarded."""
        # Large arc with r ≈ 0.5 m — well above the 0.15 m limit
        angles = np.linspace(0, math.pi / 2, 20)
        pts = np.column_stack([2.0 + 0.5 * np.cos(angles),
                               0.5 * np.sin(angles)])
        result = cluster_and_fit(pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES,
                                 CONE_R_MIN, CONE_R_MAX)
        assert result == []

    def test_single_valid_cone_detected(self):
        ranges = _make_ranges((2.0, 0.0, 0.10))
        detected = _run_pipeline(ranges)
        assert len(detected) == 1
        cone = detected[0]
        assert math.hypot(cone['cx'] - 2.0, cone['cy'] - 0.0) < POSITION_TOLERANCE_M
        assert CONE_R_MIN <= cone['radius'] <= CONE_R_MAX
        assert 0.0 <= cone['confidence'] <= 1.0

    def test_noise_only_scan_produces_no_cones(self):
        """Isolated single-beam spikes cannot form clusters of min_samples=3."""
        rng = np.random.default_rng(0)
        ranges = [float('inf')] * NUM_BEAMS
        # Scatter 30 individual beams — each isolated, never 3 within eps
        for i in rng.choice(NUM_BEAMS, size=30, replace=False):
            ranges[int(i)] = float(rng.uniform(0.5, 4.0))
        detected = _run_pipeline(ranges)
        # Isolated points cannot form clusters → either 0 or only noise-labelled
        # clusters that don't pass the radius gate.  Either way, ≤ 0 valid cones
        # is the expected outcome; allow 1 in the unlikely event two spikes land
        # within eps and a third is nearby, but certainly not more.
        assert len(detected) <= 1


# ---------------------------------------------------------------------------
# Integration test — 3 cones at known positions
# ---------------------------------------------------------------------------

# Ground truth: (cx, cy, radius_m)
# Cones are placed at least 0.6 m apart so DBSCAN never bridges two clusters.
CONE_A = (1.50,  0.00, 0.10)   # straight ahead at 1.5 m
CONE_B = (2.00,  0.70, 0.09)   # ahead-left at ~2.1 m
CONE_C = (1.20, -0.60, 0.11)   # ahead-right at ~1.3 m

GROUND_TRUTH = [(cx, cy) for cx, cy, _ in [CONE_A, CONE_B, CONE_C]]


class TestThreeConesIntegration:

    @pytest.fixture(scope='class')
    def detected(self):
        """Run the full filter → cluster → fit pipeline once for all tests."""
        ranges = _make_ranges(CONE_A, CONE_B, CONE_C)
        return _run_pipeline(ranges)

    def test_exactly_three_candidates_detected(self, detected):
        assert len(detected) == 3, (
            f'Expected 3 cone candidates, got {len(detected)}: {detected}'
        )

    def test_all_candidates_within_50mm_of_ground_truth(self, detected):
        matches = _match_detections(detected, GROUND_TRUTH, POSITION_TOLERANCE_M)
        assert len(matches) == 3, (
            f'Only {len(matches)}/3 ground-truth cones matched within '
            f'{POSITION_TOLERANCE_M * 1000:.0f} mm.\n'
            f'Detected: {[(round(c["cx"],3), round(c["cy"],3)) for c in detected]}\n'
            f'Ground truth: {GROUND_TRUTH}'
        )

    def test_cone_a_position(self, detected):
        matches = _match_detections(detected, [GROUND_TRUTH[0]], POSITION_TOLERANCE_M)
        assert len(matches) == 1
        cone, (gx, gy) = matches[0]
        assert math.hypot(cone['cx'] - gx, cone['cy'] - gy) < POSITION_TOLERANCE_M

    def test_cone_b_position(self, detected):
        matches = _match_detections(detected, [GROUND_TRUTH[1]], POSITION_TOLERANCE_M)
        assert len(matches) == 1
        cone, (gx, gy) = matches[0]
        assert math.hypot(cone['cx'] - gx, cone['cy'] - gy) < POSITION_TOLERANCE_M

    def test_cone_c_position(self, detected):
        matches = _match_detections(detected, [GROUND_TRUTH[2]], POSITION_TOLERANCE_M)
        assert len(matches) == 1
        cone, (gx, gy) = matches[0]
        assert math.hypot(cone['cx'] - gx, cone['cy'] - gy) < POSITION_TOLERANCE_M

    def test_all_radii_within_valid_range(self, detected):
        for cone in detected:
            assert CONE_R_MIN <= cone['radius'] <= CONE_R_MAX, (
                f"Radius {cone['radius']:.4f} outside [{CONE_R_MIN}, {CONE_R_MAX}]"
            )

    def test_all_confidences_are_valid(self, detected):
        for cone in detected:
            assert 0.0 <= cone['confidence'] <= 1.0

    def test_no_duplicate_detections(self, detected):
        """No two detections should be closer than cone separation distance."""
        min_gt_separation = min(
            math.hypot(GROUND_TRUTH[i][0] - GROUND_TRUTH[j][0],
                       GROUND_TRUTH[i][1] - GROUND_TRUTH[j][1])
            for i in range(len(GROUND_TRUTH))
            for j in range(i + 1, len(GROUND_TRUTH))
        )
        for i, a in enumerate(detected):
            for j, b in enumerate(detected):
                if i >= j:
                    continue
                dist = math.hypot(a['cx'] - b['cx'], a['cy'] - b['cy'])
                assert dist > min_gt_separation / 2, (
                    f'Detections {i} and {j} are suspiciously close ({dist:.3f} m) '
                    f'— possible duplicate'
                )

    def test_synthetic_scan_has_enough_returns_per_cone(self):
        """Sanity-check the test fixture: each cone should produce ≥ 5 beam hits."""
        for cx, cy, r in [CONE_A, CONE_B, CONE_C]:
            hits = _cylinder_hits(cx, cy, r)
            assert len(hits) >= 5, (
                f'Cone at ({cx}, {cy}) only produced {len(hits)} LiDAR returns — '
                f'not enough for reliable DBSCAN clustering'
            )
