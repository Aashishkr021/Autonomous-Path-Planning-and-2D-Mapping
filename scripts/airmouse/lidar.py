"""Lidar helpers: clean a scan, move it to the drone centre."""

import numpy as np


def scan_arrays(scan):
    """Sanitised (ranges, beam angles). REP-117: +inf = nothing within range, -inf = closer than range_min,
    NaN = invalid. +inf -> range_max; -inf and too-close readings -> range_min (an obstacle IS there);
    NaN -> range_max; everything is clipped to [range_min, range_max]."""
    r = np.asarray(scan.ranges, dtype=float)
    too_close = np.isneginf(r) | (np.isfinite(r) & (r < scan.range_min))
    r = np.where(np.isnan(r) | np.isposinf(r), scan.range_max, r)
    r = np.where(too_close, scan.range_min, r)
    r = np.clip(r, scan.range_min, scan.range_max)
    if scan.ang is not None:
        ang = np.asarray(scan.ang, dtype=float)
    else:
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_inc
    return r, ang


def scan_to_base(ranges, angle_min, angle_inc, off_x=0.0, off_y=0.0, off_yaw=0.0):
    """Express a scan taken from a lidar mounted at (off_x, off_y, off_yaw) in base_link, as ranges and beam
    angles measured from the base_link origin (what the safety filter needs). Special values are kept:
    +inf / -inf / NaN beams only get their direction rotated. With a centred, unrotated lidar this is a no-op."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc + off_yaw
    if abs(off_x) < 1e-6 and abs(off_y) < 1e-6:
        return r, a
    fin = np.isfinite(r) & (r > 0)
    px = off_x + np.where(fin, r, 0.0) * np.cos(a)
    py = off_y + np.where(fin, r, 0.0) * np.sin(a)
    r2 = np.where(fin, np.hypot(px, py), r)
    a2 = np.where(fin, np.arctan2(py, px), a)
    return r2, a2
