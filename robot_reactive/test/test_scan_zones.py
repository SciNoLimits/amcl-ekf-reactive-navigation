"""Unit tests for the LiDAR zone maths, using made-up scans."""
import math

import pytest

from robot_reactive.scan_zones import ZoneConfig, Zones, classify, compute_zones

N = 360
ANGLE_MIN = -math.pi
INC = 2 * math.pi / N
CFG = ZoneConfig()


def scan_with(points):
    """Build a 360-ray scan with no returns except {bearing_deg: range}."""
    ranges = [float('inf')] * N
    for deg, r in points.items():
        ranges[int(round((math.radians(deg) - ANGLE_MIN) / INC))] = r
    return ranges


def zones(points):
    return compute_zones(scan_with(points), ANGLE_MIN, INC, 0.12, 3.5, CFG)


def test_empty_scan_is_free_everywhere():
    assert zones({}) == Zones(CFG.max_range, CFG.max_range, CFG.max_range)


def test_obstacle_dead_ahead_measured_from_bumper():
    assert zones({0: 0.50}).front == pytest.approx(0.50 - CFG.lidar_to_front, abs=1e-6)


def test_nan_and_out_of_range_are_ignored():
    ranges = scan_with({0: float('nan'), 1: 0.05, 2: 9.0})
    assert compute_zones(ranges, ANGLE_MIN, INC, 0.12, 3.5, CFG).front == CFG.max_range


def test_wall_beside_robot_is_not_ahead():
    # A wall 0.4 m to the left, seen at 78 degrees (level with the body side),
    # is outside the corridor and its clearance is measured from the wheel edge.
    z = zones({78: 0.4 / math.sin(math.radians(78))})
    assert z.front == CFG.max_range
    assert z.left == pytest.approx(0.4 - CFG.half_width, abs=1e-6)
    assert z.right == CFG.max_range


def test_right_side_detected():
    z = zones({-45: 0.45})
    assert z.right < CFG.max_range and z.left == CFG.max_range


def test_point_in_corridor_counts_only_as_front():
    z = zones({25: 0.35})   # y = 0.148, inside the corridor
    assert z.front < CFG.max_range and z.left == CFG.max_range


def test_nearest_point_wins():
    assert zones({0: 1.0, 2: 0.6}).front == pytest.approx(0.6 * math.cos(math.radians(2)) - 0.093,
                                                          abs=1e-3)


@pytest.mark.parametrize('z,expected', [
    (Zones(2, 2, 2), 'CLEAR'),
    (Zones(0.2, 2, 2), 'FRONT'),
    (Zones(2, 0.1, 2), 'LEFT'),
    (Zones(2, 2, 0.1), 'RIGHT'),
    (Zones(2, 0.1, 0.1), 'BOTH'),
    (Zones(0.2, 0.1, 2), 'FRONT'),
])
def test_classify(z, expected):
    assert classify(z, 0.35, 0.25) == expected
