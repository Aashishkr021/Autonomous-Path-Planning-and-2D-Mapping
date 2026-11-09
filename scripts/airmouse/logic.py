"""Mission state machine (ENTER > EXPLORE > SWEEP > RETURN > HOME > DONE). Pure Python, no ROS."""
import math
from dataclasses import dataclass

import numpy as np
from .lidar import scan_arrays
from .mapgrid import cells_world, in_bounds, prepare, to_cell, to_world
from .params import DEFAULTS
from .pathplan import astar_path, detect_rooms, dijkstra, extract_path, pick_frontier, plan_tour, unvisited_targets
from .safety import los_free, safety_filter, smooth_path


@dataclass
class Cmd:
    vx: float = 0.0   # body frame, ROS convention: x forward, y left (m/s)
    vy: float = 0.0
    state: str = 'EXPLORE'


class Logic:
    """State machine: ENTER -> EXPLORE (frontiers) -> SWEEP (visit every room) -> RETURN -> HOME -> DONE."""

    def __init__(self, **kw):
        self.p = {**DEFAULTS, **kw}
        # the planner must never route through gaps the lidar safety bubble would refuse to enter
        self.p['inflate_m'] = max(self.p['inflate_m'], self.p['safe_radius_m'] + 0.03)
        self.state = 'ENTER' if self.p['entry_forward_m'] > 0 else 'EXPLORE'
        self.goal = None
        self.path = []
        self.blacklist = []
        self.empty = 0
        self.n_frontiers = 0
        self.last_plan = -1e9
        self.plan_id = 0
        self._start = None
        self.ref_pos = None
        self.ref_t = 0.0
        self.home = None
        self.home_yaw = 0.0
        self.trail = []
        self.tour = []
        self.room_reg = []     # persistent rooms: dict(id, pt, visited). Never forgotten, never duplicated.
        self.msgs = []         # human-readable events, printed by the node
        self.leg_t0 = None
        self.entered = self.state != 'ENTER'
        self.sweep_kind = 'room'
        self.next_room_id = 1
        self.t_start = None
        self.ret_t0 = None
        self.reason = ''
        self.g = None            # last prepared planning grid (used for look-ahead collision checks)
        self.v_prev = np.zeros(2)  # last commanded body-frame velocity (for the acceleration limit)
        self.t_prev = None
        self.home_planned = False
        self.ret_stuck = 0
        self.ret_wait0 = None

    def _skip_goal(self):
        if self.goal:
            self.blacklist.append(self.goal)
        self.goal, self.path = None, []
        self.last_plan = -1e9

    def _set_leg(self, a, b):
        """Straight path a -> b, a point every 0.2 m."""
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.2) + 1)
        self.path = [(a[0] + (b[0] - a[0]) * k / (n - 1), a[1] + (b[1] - a[1]) * k / (n - 1))
                     for k in range(n)]
        self.goal = b

    def _finish_path(self, g, par, cell):
        pts = None
        if self.p['planner'] == 'astar' and self._start is not None:
            pts = astar_path(g, self._start, cell)          # A*: start -> chosen goal
        if not pts:                                         # Dijkstra route (also the safety fallback)
            pts = extract_path(g, par, cell)
        return smooth_path(g, pts, max(self.p['smooth_clear_m'], self.p['inflate_m']))

    def _final_approach_clear(self, px, py, margin=0.3):
        """Short straight last leg to the launch point: no known obstacle within `margin` of the segment."""
        g = self.g
        if g is None or math.hypot(self.home[0] - px, self.home[1] - py) > self.p['final_approach_max_m']:
            return False
        n = max(2, int(math.hypot(self.home[0] - px, self.home[1] - py) / (g['cs'] * 0.5)) + 1)
        for x, y in zip(np.linspace(px, self.home[0], n), np.linspace(py, self.home[1], n)):
            i, j = to_cell(g, x, y)
            if in_bounds(g, i, j) and g['d'][i, j] < margin:
                return False
        return True

    def _replan(self, px, py, grid):
        p = self.p
        gate = None
        if p['keep_inside'] and p['entry_forward_m'] > 0 and self.state in ('EXPLORE', 'SWEEP'):
            gate = (self.home[0], self.home[1], math.cos(self.home_yaw), math.sin(self.home_yaw), p['gate_m'])
        g = prepare(grid, p, gate)
        if g is None:
            return
        i, j = to_cell(g, px, py)
        if not in_bounds(g, i, j) or g['occ'][i, j]:
            return
        # The drone may always leave the cell it stands in, even when that cell is inside the inflated
        # zone: allow the start cell, plus KNOWN-FREE neighbours that are not tighter than the start cell.
        # Never occupied cells, never unknown cells.
        g['trav'][i, j] = True
        d_here = g['d'][i, j]
        for di in (-2, -1, 0, 1, 2):
            for dj in (-2, -1, 0, 1, 2):
                a, b = i + di, j + dj
                if in_bounds(g, a, b) and not g['occ'][a, b] and not g['unk'][a, b] \
                        and g['d'][a, b] >= min(p['inflate_m'], d_here):
                    g['trav'][a, b] = True
        g.pop('graph', None)
        self.g = g
        self._start = (i, j)
        dist, par = dijkstra(g, (i, j))      # cost to every cell: used only to CHOOSE the goal
        self.plan_id += 1
        if self.state in ('EXPLORE', 'SWEEP') and p['sweep_rooms']:
            hub = None
            if p['entry_forward_m'] > 0 and self.home is not None:
                hub = (self.home[0] + math.cos(self.home_yaw) * (p['gate_m'] + 0.5),
                       self.home[1] + math.sin(self.home_yaw) * (p['gate_m'] + 0.5))
            for pt in detect_rooms(g, p, [r_['pt'] for r_ in self.room_reg], hub):
                self.room_reg.append(dict(id=self.next_room_id, pt=pt, visited=False))
                self.msgs.append('Room R%d found at (%.1f, %.1f)' % (self.next_room_id, pt[0], pt[1]))
                self.next_room_id += 1
            if self.trail:
                tr_ = np.array(self.trail)
                for r_ in self.room_reg:
                    if not r_['visited'] and np.min(np.hypot(tr_[:, 0] - r_['pt'][0], tr_[:, 1] - r_['pt'][1])) <= p['room_visit_m']:
                        r_['visited'] = True
                        self.msgs.append('Room R%d visited (%d of %d)' % (
                            r_['id'], sum(x['visited'] for x in self.room_reg), len(self.room_reg)))
        if self.state == 'EXPLORE':
            cell, n = pick_frontier(g, dist, self.blacklist, self.goal, p)
            self.n_frontiers = n
            if cell is None:
                self.empty += 1
                self.goal, self.path = None, []
                if self.empty >= p['finish_after_n_empty']:
                    self.state = 'SWEEP' if p['sweep_rooms'] else ('RETURN' if p['return_home'] else 'DONE')
                    self.empty = 0
                    self.last_plan = -1e9
                return
            self.empty = 0
            self.path = self._finish_path(g, par, cell)
            self.goal = self.path[-1]
        elif self.state == 'SWEEP':
            targets = []
            for r_ in self.room_reg:
                pt = r_['pt']
                if r_['visited']:
                    continue                                   # drone has been inside already
                if any(math.hypot(pt[0] - b[0], pt[1] - b[1]) < 0.5 for b in self.blacklist):
                    continue
                c = to_cell(g, *pt)
                if in_bounds(g, *c) and np.isfinite(dist[c]):
                    targets.append(c)
            self.n_frontiers = len(targets)
            if not targets and p['cover_sweep']:
                cells = unvisited_targets(g, dist, self.trail, self.blacklist,
                                          {**p, 'visit_radius_m': p['cover_radius_m'],
                                           'min_sweep_cells': p['cover_min_cells']})
                if cells:
                    self.sweep_kind = 'cover'
                    self.n_frontiers = len(cells)
                    self.empty = 0
                    pts_c = [to_world(g, *c) for c in cells]
                    keep = None
                    if self.goal is not None:                       # stick with the goal while some uncovered area is still near it
                        if any(math.hypot(self.goal[0] - q[0], self.goal[1] - q[1]) <= 1.5 for q in pts_c):
                            keep = to_cell(g, *self.goal)
                    if keep is None or not (in_bounds(g, *keep) and np.isfinite(dist[keep])):
                        keep = min(cells, key=lambda c: dist[c])
                    self.path = self._finish_path(g, par, keep)
                    self.goal = self.path[-1]
                    self.tour = []
                    return
            self.sweep_kind = 'room'
            if not targets:
                self.empty += 1
                self.goal, self.path, self.tour = None, [], []
                if self.empty >= p['finish_after_n_empty']:
                    self.state = 'RETURN' if p['return_home'] else 'DONE'
                    self.last_plan = -1e9
                return
            self.empty = 0
            cur = [to_world(g, *c) for c in targets]
            # keep the planned order; drop stops that are done; rebuild only for a genuinely new area
            tour = []
            for pt in self.tour:
                k = min(range(len(cur)), key=lambda q: math.hypot(cur[q][0] - pt[0], cur[q][1] - pt[1]))
                if math.hypot(cur[k][0] - pt[0], cur[k][1] - pt[1]) <= 2.0 and cur[k] not in tour:
                    tour.append(cur[k])
            new_area = any(min((math.hypot(c[0] - t_[0], c[1] - t_[1]) for t_ in tour), default=1e9) > 2.0
                           for c in cur)
            if not tour or new_area:
                ii, jj = np.nonzero(np.isfinite(dist))
                wx, wy = cells_world(g, ii, jj)
                d_home = np.hypot(wx - self.home[0], wy - self.home[1])
                kx = int(np.argmin(d_home))
                order = plan_tour(g, (i, j), targets, (int(ii[kx]), int(jj[kx])))
                tour = [to_world(g, *c) for c in order]
            self.tour = tour
            self.path = self._finish_path(g, par, to_cell(g, *tour[0]))
            self.goal = self.path[-1]
        else:  # RETURN: plan the whole way back to the launch point itself
            hc = to_cell(g, *self.home)
            self.home_planned = bool(in_bounds(g, *hc) and np.isfinite(dist[hc]))
            if self.home_planned:
                cell = hc
            else:                          # home unknown / blocked: nearest SAFE reachable cell instead
                ii, jj = np.nonzero(np.isfinite(dist))
                if len(ii) == 0:
                    self.reason = 'no reachable cell - landing here'
                    self.state = 'DONE'
                    return
                wx, wy = cells_world(g, ii, jj)
                score = np.hypot(wx - self.home[0], wy - self.home[1]) + \
                    1.0 * (g['d'][ii, jj] < max(p['pref_land_clear_m'], p['inflate_m']))
                k = int(np.argmin(score))
                cell = (int(ii[k]), int(jj[k]))
            self.path = self._finish_path(g, par, cell)
            self.goal = self.path[-1]

    def update(self, t, pose, scan, grid):
        p = self.p
        px, py, yaw = pose
        if self.home is None:
            self.home = (px, py)
            self.home_yaw = yaw
            if self.state == 'ENTER':
                f = (math.cos(yaw), math.sin(yaw))
                self._set_leg(self.home, (px + f[0] * p['entry_forward_m'], py + f[1] * p['entry_forward_m']))
                self.leg_t0 = t
        if self.t_start is None:
            self.t_start = t
        el = t - self.t_start
        if self.state not in ('DONE', 'HOME'):
            if el > p['mission_limit_s']:
                self.reason = 'mission time limit - landing where the drone is'
                self.state = 'DONE'
            elif self.state in ('EXPLORE', 'SWEEP') and el > p['explore_limit_s']:
                self.reason = 'explore time limit - going home'
                self.state, self.goal, self.path, self.last_plan = 'RETURN', None, [], -1e9
            if self.state == 'RETURN':
                if self.ret_t0 is None:
                    self.ret_t0 = t
                elif t - self.ret_t0 > p['return_limit_s']:
                    self.reason = 'could not get home in time - landing where the drone is'
                    self.state = 'DONE'
        r, ang = scan_arrays(scan)
        if not self.trail or math.hypot(px - self.trail[-1][0], py - self.trail[-1][1]) > 0.15:
            self.trail.append((px, py))

        if self.state == 'DONE':
            return Cmd(0.0, 0.0, 'DONE')

        if self.state == 'ENTER':     # straight line through the entrance, no planning
            if math.hypot(self.goal[0] - px, self.goal[1] - py) < 0.4 or t - self.leg_t0 > p['enter_timeout_s']:
                self.state, self.entered = 'EXPLORE', True
                self.goal, self.path, self.last_plan = None, [], -1e9
        elif self.state == 'HOME':    # final straight approach onto the launch pad
            if math.hypot(self.goal[0] - px, self.goal[1] - py) < 0.15 or t - self.leg_t0 > 60.0:
                self.state = 'DONE'
                return Cmd(0.0, 0.0, 'DONE')

        # progress monitor
        if self.ref_pos is None or math.hypot(px - self.ref_pos[0], py - self.ref_pos[1]) > 0.3:
            self.ref_pos, self.ref_t = (px, py), t
        elif self.goal is not None and self.state in ('EXPLORE', 'SWEEP', 'RETURN') and t - self.ref_t > p['stuck_timeout_s']:
            self._skip_goal()
            self.ref_t = t
            if self.state == 'RETURN':
                self.ret_stuck += 1
                if self.ret_stuck >= p['ret_stuck_max']:
                    self.reason = 'stuck on the way home - landing where the drone is'
                    self.state = 'DONE'
                    return Cmd(0.0, 0.0, 'DONE')

        # goal reached
        if self.state in ('EXPLORE', 'SWEEP', 'RETURN') and self.goal and \
                math.hypot(self.goal[0] - px, self.goal[1] - py) < (
                    (p['cover_goal_tol_m'] if self.sweep_kind == 'cover' else p['sweep_goal_tol_m'])
                    if self.state == 'SWEEP' else p['goal_tol_m']):
            if self.state == 'RETURN':
                if self.home_planned and math.hypot(self.home[0] - px, self.home[1] - py) < max(p['home_tol_m'], 0.35):
                    self.reason = 'home reached'
                    self.state = 'DONE'
                    return Cmd(0.0, 0.0, 'DONE')
                if self.ret_wait0 is None:
                    self.ret_wait0 = t
                waited = t - self.ret_wait0
                if self._final_approach_clear(px, py, 0.3 if waited < p['return_retry_s'] / 2 else 0.2):
                    self.state, self.leg_t0 = 'HOME', t
                    self._set_leg((px, py), self.home)
                elif waited < p['return_retry_s']:             # map may still change as the drone gets closer: wait and re-plan
                    self.goal, self.path, self.last_plan = None, [], -1e9
                else:
                    self.reason = 'launch point not reachable - landing at the nearest safe point'
                    self.state = 'DONE'
                    return Cmd(0.0, 0.0, 'DONE')
            elif self.state == 'SWEEP':   # visited now -> no blacklist, just pick the next area
                self.goal, self.path, self.last_plan = None, [], -1e9
            else:
                self._skip_goal()

        if self.state in ('EXPLORE', 'SWEEP', 'RETURN') and t - self.last_plan >= p['replan_period_s']:
            self.last_plan = t
            self._replan(px, py, grid)
            if self.state == 'DONE':
                return Cmd(0.0, 0.0, 'DONE')

        # ---- desired velocity from the path ------------------------------------------------
        dt = 0.1 if self.t_prev is None else float(np.clip(t - self.t_prev, 0.02, 0.5))
        self.t_prev = t
        v = np.zeros(2)
        if self.path:
            pts = np.array(self.path)
            k = int(np.argmin(np.hypot(pts[:, 0] - px, pts[:, 1] - py)))
            # look-ahead grows with speed and shrinks near obstacles
            look = float(np.clip(p['lookahead_min_m'] + 0.8 * float(np.hypot(*self.v_prev)),
                                 p['lookahead_min_m'], p['lookahead_m']))
            look = min(look, max(p['lookahead_min_m'], 0.6 * float(np.min(r))))
            kt, s_acc = len(pts) - 1, 0.0
            for m in range(k, len(pts) - 1):
                s_acc += float(np.hypot(*(pts[m + 1] - pts[m])))
                if s_acc >= look:
                    kt = m + 1
                    break
            # never cut a corner: shorten the look-ahead until the straight line to it is collision free
            if self.g is not None and self.state in ('EXPLORE', 'SWEEP', 'RETURN'):
                while kt > k + 1 and not los_free(self.g, (px, py), tuple(pts[kt]), self.g['trav'], skip_m=0.35):
                    kt -= 1
            tgt = pts[kt]
            d = np.array([tgt[0] - px, tgt[1] - py])
            n = float(np.hypot(*d))
            if n > 1e-3:
                d_goal = math.hypot(pts[-1][0] - px, pts[-1][1] - py)
                speed = min(p['cruise_speed'] if self.state != 'HOME' else 0.25, max(0.12, 0.7 * d_goal))
                vw = d / n * speed
                c, s_ = math.cos(yaw), math.sin(yaw)
                v = np.array([c * vw[0] + s_ * vw[1], -s_ * vw[0] + c * vw[1]])

        # acceleration limit (no abrupt velocity steps) ...
        dv = v - self.v_prev
        dn = float(np.hypot(*dv))
        if dn > p['accel_limit'] * dt:
            v = self.v_prev + dv / dn * (p['accel_limit'] * dt)
        # ... then the lidar barrier LAST, so its guarantee holds for what is actually sent
        v = safety_filter(v, r, ang, p['safe_radius_m'], p['reaction_s'], p['brake_decel'])
        n = float(np.hypot(*v))
        if n > p['max_speed']:
            v *= p['max_speed'] / n
        self.v_prev = v.copy()
        return Cmd(float(v[0]), float(v[1]), self.state)
