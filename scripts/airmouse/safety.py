"""Safety: stopping-distance filter, line-of-sight check, path smoothing, altitude speed factor."""
import math

import numpy as np


def alt_speed_factor(err, free=0.10, span=0.15):
    """Horizontal speed multiplier from the altitude error: full speed inside +-free, zero beyond free+span.
    (The drone first fixes its height, then moves on.)"""
    return float(np.clip(1.0 - (abs(err) - free) / span, 0.0, 1.0))


def safety_filter(v, r, ang, safe_radius, reaction_s, decel, retreat=0.25):
    """Lidar barrier on the commanded velocity v (body frame, x forward, y left).
    Outside the safety radius the speed towards each beam must allow the drone to stop before it:
        v * reaction_s + v^2 / (2 * decel)  <=  range - safe_radius
    (reaction_s covers command latency + velocity lag, decel the braking).
    Inside the safety radius the drone may not get any closer to that obstacle (it can slide along it)
    and is pushed gently away (soft, max `retreat` m/s) - so it can still crawl out of a tight squeeze.
    The returned velocity ALWAYS satisfies every constraint (it is scaled down if needed)."""
    u = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    d = r - safe_radius
    a_pos = decel * (-reaction_s + np.sqrt(reaction_s ** 2 + 2.0 * np.maximum(d, 0.0) / decel))
    allowed = np.where(d >= 0.0, a_pos, 0.0)
    v = np.array(v, dtype=float)
    inside = d < 0.0
    if inside.any():                                   # soft push away from whatever is too close
        push = -(u[inside] * (-d[inside])[:, None]).sum(axis=0)
        n = float(np.hypot(*push))
        if n > 1e-6:
            v = v + push / n * min(retreat, 2.0 * n)
    for _ in range(12):
        viol = u @ v - allowed
        k = int(np.argmax(viol))
        if viol[k] <= 1e-3:
            return v
        v = v - viol[k] * u[k]
    proj = u @ v
    bad = proj > allowed + 1e-3                        # not converged: shrink v until every constraint holds
    if bad.any():
        alpha = float(np.min(np.where(proj[bad] > 1e-9, allowed[bad] / proj[bad], 1.0)))
        v = v * max(0.0, min(1.0, alpha))
    return v


def los_free(g, a, b, mask, skip_m=0.0):
    """True if the straight segment a->b (world coords) stays on True cells of `mask`.
    Samples within skip_m of `a` are ignored (the drone may stand on a not-quite-free cell)."""
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (g['cs'] * 0.5)) + 1)
    xs, ys = np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)
    dx, dy = xs - g['ox'], ys - g['oy']
    jj = np.floor((g['cy'] * dx + g['sy'] * dy) / g['cs']).astype(int)
    ii = np.floor((-g['sy'] * dx + g['cy'] * dy) / g['cs']).astype(int)
    inb = (ii >= 0) & (ii < g['h']) & (jj >= 0) & (jj < g['w'])
    ok = np.zeros(n, bool)
    ok[inb] = mask[ii[inb], jj[inb]]
    return bool((ok | (np.hypot(xs - a[0], ys - a[1]) < skip_m)).all())


def smooth_path(g, pts, clear_m, step=0.2, window=25):
    """Remove staircase turns: keep only the waypoints needed so that every straight piece keeps
    `clear_m` clearance on known-free cells, then re-sample every `step` metres."""
    if len(pts) < 3:
        return pts
    mask = g['trav'] & (g['d'] >= clear_m)
    keep, i, n = [pts[0]], 0, len(pts)
    while i < n - 1:
        j = min(n - 1, i + window)
        while j > i + 1 and not los_free(g, pts[i], pts[j], mask, skip_m=0.3 if i == 0 else 0.0):
            j -= 1
        keep.append(pts[j])
        i = j
    out = [keep[0]]
    for a, b in zip(keep[:-1], keep[1:]):
        k = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        out += [(a[0] + (b[0] - a[0]) * m / k, a[1] + (b[1] - a[1]) * m / k) for m in range(1, k + 1)]
    return out
