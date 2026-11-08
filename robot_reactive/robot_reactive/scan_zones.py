"""Turn one LiDAR scan into three clearance numbers: front, left and right.

This file has no ROS imports, so the maths can be unit-tested on its own.

Coordinates are in the LiDAR frame (``base_scan``): +x forward, +y left.
Every distance returned is measured from the *surface of the robot body*, not
from the LiDAR, because the body is what would actually hit something.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class ZoneConfig:
    # Robot body as seen from the LiDAR (from turtlebot3_waffle_pi.urdf.xacro):
    # chassis x spans [-0.197, 0.069] in base_link, the LiDAR is at x = -0.024,
    # and the wheels reach y = +/-0.153.
    lidar_to_front: float = 0.093
    lidar_to_rear: float = 0.173
    half_width: float = 0.153
    # Extra width each side of the body for the "straight ahead" corridor.
    corridor_margin: float = 0.05
    # Bearing range, either side of straight ahead, of the side sectors.
    side_angle_min_deg: float = 20.0
    side_angle_max_deg: float = 80.0
    # Anything further than this counts as "free"; also the value reported
    # when a zone contains no points at all.
    max_range: float = 2.0


@dataclass
class Zones:
    front: float
    left: float
    right: float


def _body_distance(x: np.ndarray, y: np.ndarray, cfg: ZoneConfig) -> np.ndarray:
    """Shortest distance from each point to the robot's body rectangle."""
    dx = np.maximum(np.maximum(-cfg.lidar_to_rear - x, x - cfg.lidar_to_front), 0.0)
    dy = np.maximum(np.abs(y) - cfg.half_width, 0.0)
    return np.hypot(dx, dy)


def compute_zones(ranges, angle_min, angle_increment, range_min, range_max,
                  cfg: ZoneConfig) -> Zones:
    """Return front/left/right clearance for a LaserScan's data.

    Front uses a rectangle (a corridor as wide as the robot) instead of an
    angular cone. A cone also catches walls beside the robot in a doorway; a
    rectangle only catches what the robot would drive into if it went straight.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(r.size) * angle_increment

    # Rays with no return are inf or NaN, or fall outside the sensor's limits.
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, angles = r[valid], angles[valid]
    x, y = r * np.cos(angles), r * np.sin(angles)
    dist = _body_distance(x, y, cfg)

    in_corridor = (x > cfg.lidar_to_front) & (np.abs(y) <= cfg.half_width + cfg.corridor_margin)

    amin, amax = np.radians(cfg.side_angle_min_deg), np.radians(cfg.side_angle_max_deg)
    # A point already in the corridor is a "front" obstacle, not a side one.
    in_left = (angles >= amin) & (angles <= amax) & ~in_corridor
    in_right = (angles <= -amin) & (angles >= -amax) & ~in_corridor

    def clearance(mask: np.ndarray) -> float:
        return float(min(dist[mask].min(), cfg.max_range)) if mask.any() else cfg.max_range

    return Zones(clearance(in_corridor), clearance(in_left), clearance(in_right))


def classify(zones: Zones, stop_distance: float, side_distance: float) -> str:
    """Name the situation, for display and logging: CLEAR/FRONT/LEFT/RIGHT/BOTH."""
    if zones.front < stop_distance:
        return 'FRONT'
    left_close, right_close = zones.left < side_distance, zones.right < side_distance
    if left_close and right_close:
        return 'BOTH'
    if left_close:
        return 'LEFT'
    if right_close:
        return 'RIGHT'
    return 'CLEAR'
