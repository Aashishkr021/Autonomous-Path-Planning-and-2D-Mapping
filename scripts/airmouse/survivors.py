"""Survivor tagging maths (no ROS): camera box -> metres -> map position, distinct-person tracker, grid labels."""
import math

import numpy as np

SURVIVOR_DEFAULTS = dict(
    img_w=640, img_h=480, hfov=1.396,    # camera model (model.sdf)
    cam_x=0.12, cam_h_m=0.45,            # camera is 0.12 m ahead of base_link, ~0.45 m above the floor in flight
    min_range_m=0.6, max_range_m=6.0,    # ignore people closer / farther than this
    lidar_tol_m=0.6,                     # lidar range replaces the camera range if they agree this well
    person_radius_m=0.15,                # lidar hits the front of the body, the centre is this much farther
    assoc_m=0.9, merge_m=1.0,            # detections closer than this are the SAME person
    min_hits=5, min_span_s=0.5,          # a person is confirmed after this many sightings spread over this time
    max_survivors=6,
    grid_m=3.0, grid_x0=-7.5, grid_y0=0.5,   # grid box size and the south-west corner (map frame) of the building
    grid_cols=5, grid_rows=5,
)


def box_to_base(box, p):
    """Pixel box (x1,y1,x2,y2) -> (x_forward, y_left, method) in the drone frame, or None.

    Level camera at height h: a floor point seen `dv` pixels below the image centre is h*f/dv metres ahead.
    The bottom edge of the box is where the person's feet touch the floor."""
    x1, y1, x2, y2 = box[:4]
    w, hgt = p['img_w'], p['img_h']
    f = (w / 2.0) / math.tan(p['hfov'] / 2.0)
    cx, cy = w / 2.0, hgt / 2.0
    if y2 >= hgt - 4 or y2 <= cy + 3:           # feet cut off by the image edge, or above the horizon
        return None
    depth = p['cam_h_m'] * f / (y2 - cy)
    if not (p['min_range_m'] <= depth <= p['max_range_m']):
        return None
    u = 0.5 * (x1 + x2)
    left = -(u - cx) / f * depth
    return p['cam_x'] + depth, left, 'camera'


def lidar_range_at(ranges, angle_min, angle_inc, bearing, half_width):
    """Nearest valid lidar range within +-half_width (rad) of `bearing` (rad, left positive), or None."""
    n = len(ranges)
    if n == 0 or angle_inc == 0:
        return None
    a = bearing - angle_min
    k0 = int(round((a - half_width) / angle_inc))
    k1 = int(round((a + half_width) / angle_inc))
    vals = [ranges[k % n] for k in range(min(k0, k1), max(k0, k1) + 1)]
    vals = [v for v in vals if v is not None and math.isfinite(v) and v > 0.05]
    return min(vals) if vals else None


def refine_with_lidar(xy, box, scan, p):
    """If the lidar saw something at about the same range, use its (more exact) range along the same bearing."""
    if scan is None:
        return xy, 'camera'
    x, y = xy
    ranges, a0, da = scan
    bearing = math.atan2(y, x)
    half_w = max(math.radians(2.0), 0.4 * (box[2] - box[0]) / (p['img_w'] / 2.0 / math.tan(p['hfov'] / 2.0)))
    lr = lidar_range_at(ranges, a0, da, bearing, half_w)
    r_cam = math.hypot(x, y)
    if lr is None:
        return xy, 'camera'
    r = lr + p['person_radius_m']
    if abs(r - r_cam) <= p['lidar_tol_m']:
        return (r * math.cos(bearing), r * math.sin(bearing)), 'camera+lidar'
    return xy, 'camera'


def base_to_map(pose, xy):
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * xy[0] - s * xy[1], py + s * xy[0] + c * xy[1]


def grid_label(x, y, p):
    """'A1' style box: letters go west -> east, numbers south -> north."""
    col = int(math.floor((x - p['grid_x0']) / p['grid_m']))
    row = int(math.floor((y - p['grid_y0']) / p['grid_m']))
    col = min(max(col, 0), p['grid_cols'] - 1)
    row = min(max(row, 0), p['grid_rows'] - 1)
    return '%s%d' % (chr(ord('A') + col), row + 1)


class SurvivorTracker:
    """Keeps one track per real person. A track becomes a SURVIVOR only after repeated sightings."""

    def __init__(self, p=None):
        self.p = dict(SURVIVOR_DEFAULTS, **(p or {}))
        self.tracks = []          # dict(x, y, n, t0, t1, conf, sid)
        self.next_id = 1
        self.events = []

    def _confirmed(self):
        return [t for t in self.tracks if t['sid'] is not None]

    def update(self, t, x, y, conf=1.0):
        p = self.p
        best, bd = None, 1e9
        for tr in self.tracks:
            d = math.hypot(tr['x'] - x, tr['y'] - y)
            if d < bd:
                best, bd = tr, d
        if best is None or bd > p['assoc_m']:
            best = dict(x=x, y=y, n=0, t0=t, t1=t, conf=0.0, sid=None)
            self.tracks.append(best)
        k = min(best['n'], 30)                      # running mean, newest sightings still count
        best['x'] = (best['x'] * k + x) / (k + 1)
        best['y'] = (best['y'] * k + y) / (k + 1)
        best['n'] += 1
        best['t1'] = t
        best['conf'] = max(best['conf'], conf)
        self._merge_close()
        if best in self.tracks and best['sid'] is None and best['n'] >= p['min_hits'] \
                and best['t1'] - best['t0'] >= p['min_span_s']:
            if len(self._confirmed()) >= p['max_survivors']:
                self.events.append('limit: already %d survivors, ignoring another person near (%.1f, %.1f)'
                                   % (p['max_survivors'], best['x'], best['y']))
                self.tracks.remove(best)
            else:
                best['sid'] = self.next_id
                self.next_id += 1
                self.events.append('SURVIVOR S%d confirmed at (%.2f, %.2f) box %s'
                                   % (best['sid'], best['x'], best['y'], grid_label(best['x'], best['y'], p)))

    def _merge_close(self):
        p = self.p
        changed = True
        while changed:
            changed = False
            for i, a in enumerate(self.tracks):
                for b in self.tracks[i + 1:]:
                    if math.hypot(a['x'] - b['x'], a['y'] - b['y']) < p['merge_m']:
                        keep, drop = (a, b) if (a['sid'] is not None or (b['sid'] is None and a['n'] >= b['n'])) else (b, a)
                        tot = keep['n'] + drop['n']
                        keep['x'] = (keep['x'] * keep['n'] + drop['x'] * drop['n']) / tot
                        keep['y'] = (keep['y'] * keep['n'] + drop['y'] * drop['n']) / tot
                        keep['n'] = tot
                        keep['t0'] = min(keep['t0'], drop['t0'])
                        keep['t1'] = max(keep['t1'], drop['t1'])
                        self.tracks.remove(drop)
                        changed = True
                        break
                if changed:
                    break

    def survivors(self):
        p = self.p
        return [dict(id='S%d' % t['sid'], x=round(t['x'], 2), y=round(t['y'], 2),
                     box=grid_label(t['x'], t['y'], p), sightings=t['n'], first_seen_s=round(t['t0'], 1))
                for t in sorted(self._confirmed(), key=lambda t: t['sid'])]

    def pop_events(self):
        e, self.events = self.events, []
        return e