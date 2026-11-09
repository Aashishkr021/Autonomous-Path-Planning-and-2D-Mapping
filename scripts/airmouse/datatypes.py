"""Plain data containers: Grid (SLAM map) and Scan (lidar)."""
from dataclasses import dataclass

import numpy as np


@dataclass
class Grid:
    data: np.ndarray  # int8 (H, W): -1 unknown, 0 free, 100 occupied
    res: float
    ox: float
    oy: float
    yaw: float = 0.0   # rotation of the map origin (slam_toolbox publishes 0, but other mappers may not)


@dataclass
class Scan:
    ranges: np.ndarray
    angle_min: float
    angle_inc: float
    range_min: float
    range_max: float
    ang: object = None  # optional per-beam angles in base_link (set when the lidar is mounted off-centre/rotated)
