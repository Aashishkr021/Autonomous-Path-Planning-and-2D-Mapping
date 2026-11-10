"""Path planning: A* (single goal), Dijkstra (distance to every cell), frontier picking, room detection, room tour."""
import heapq
import math

import numpy as np
from .deps import ndimage, csr_matrix, _sp_dijkstra
from .mapgrid import blank_around, cells_world, in_bounds, to_cell, to_world


def _graph(g):
    """Sparse 8-connected graph of the traversable cells (no corner cutting), cached per planning cycle."""
    if 'graph' in g:
        return g['graph']
    trav, cost = g['trav'], g['cost']
    h, w = g['h'], g['w']
    idx = np.arange(h * w).reshape(h, w)
    rows, cols, wts = [], [], []
    for di, dj, step in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, 1.4142), (1, -1, 1.4142)):
        i0, i1 = 0, h - di
        j0, j1 = (0, w - dj) if dj >= 0 else (-dj, w)
        src = (slice(i0, i1), slice(j0, j1))
        dst = (slice(i0 + di, i1 + di), slice(j0 + dj, j1 + dj))
        ok = trav[src] & trav[dst]
        if di and dj:
            ok &= trav[i0:i1, j0 + dj:j1 + dj] & trav[i0 + di:i1 + di, j0:j1]
        wt = (step * 0.5 * (cost[src] + cost[dst]))[ok]
        u, v = idx[src][ok], idx[dst][ok]
        rows += [u, v]; cols += [v, u]; wts += [wt, wt]
    g['graph'] = csr_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                            shape=(h * w, h * w))
    return g['graph']


def dijkstra_multi(g, cells):
    """Cost-to-reach every cell from each start cell in `cells`: returns (dist[m,h,w], pred[m,h*w])."""
    h, w = g['h'], g['w']
    src = [c[0] * w + c[1] for c in cells]
    if _sp_dijkstra is not None:
        dist, pred = _sp_dijkstra(_graph(g), directed=True, indices=src, return_predecessors=True)
        dist = np.atleast_2d(dist).reshape(len(src), h, w)
        pred = np.atleast_2d(pred).astype(np.int64)
        pred[pred < 0] = -1
        return dist, pred
    out_d, out_p = [], []
    for c in cells:
        d, par = _dijkstra_py(g, c)
        out_d.append(d); out_p.append(np.array(par))
    return np.array(out_d), np.array(out_p)


def dijkstra(g, start):
    """Cost-to-reach every traversable cell from `start` (8-connected, no corner cutting)."""
    d, par = dijkstra_multi(g, [start])
    return d[0], par[0]


def _dijkstra_py(g, start):
    
    h, w = g['h'], g['w']
    trav = g['trav'].ravel().tolist()
    cost = g['cost'].ravel().tolist()
    dist = [math.inf] * (h * w)
    par = [-1] * (h * w)
    s = start[0] * w + start[1]
    dist[s] = 0.0
    pq = [(0.0, s)]
    nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414))
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        ui, uj = divmod(u, w)
        for di, dj, step in nbrs:
            vi, vj = ui + di, uj + dj
            if not (0 <= vi < h and 0 <= vj < w):
                continue
            v = vi * w + vj
            if not trav[v]:
                continue
            if di and dj and not (trav[ui * w + vj] and trav[vi * w + uj]):
                continue
            nd = d + step * 0.5 * (cost[u] + cost[v])
            if nd < dist[v]:
                dist[v] = nd
                par[v] = u
                heapq.heappush(pq, (nd, v))
    return np.array(dist).reshape(h, w), par


def astar(g, start, goal):
    """A* shortest path start -> goal on the same graph/costs as Dijkstra. Returns a list of (i, j) cells or None.

    Heuristic = octile distance (8-connected, step costs 1 and 1.414). Every cell costs >= 1, so the heuristic
    never over-estimates: A* returns the same optimal cost as Dijkstra, but only searches towards the goal.
    """
    h, w = g['h'], g['w']
    trav = g['trav'].ravel().tolist()
    cost = g['cost'].ravel().tolist()
    s, t = start[0] * w + start[1], goal[0] * w + goal[1]
    if not (trav[s] and trav[t]):
        return None
    gi, gj = goal
    D, D2 = 1.0, 1.4142

    def hfun(i, j):
        dx, dy = abs(i - gi), abs(j - gj)
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    best = {s: 0.0}
    par = {s: -1}
    pq = [(hfun(*start), 0.0, s)]
    closed = set()
    nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, D2), (-1, 1, D2), (1, -1, D2), (1, 1, D2))
    while pq:
        _, gc, u = heapq.heappop(pq)
        if u in closed:
            continue
        if u == t:
            cells = []
            while u >= 0:
                cells.append(divmod(u, w))
                u = par[u]
            cells.reverse()
            return cells
        closed.add(u)
        ui, uj = divmod(u, w)
        for di, dj, step in nbrs:
            vi, vj = ui + di, uj + dj
            if not (0 <= vi < h and 0 <= vj < w):
                continue
            v = vi * w + vj
            if not trav[v] or v in closed:
                continue
            if di and dj and not (trav[ui * w + vj] and trav[vi * w + uj]):     # no corner cutting
                continue
            ng = gc + step * 0.5 * (cost[u] + cost[v])
            if ng < best.get(v, math.inf):
                best[v] = ng
                par[v] = u
                heapq.heappush(pq, (ng + hfun(vi, vj), ng, v))
    return None


def astar_path(g, start, goal):
    """A* route as world points (same format as extract_path), or None if the goal cannot be reached."""
    cells = astar(g, start, goal)
    return None if cells is None else [to_world(g, i, j) for i, j in cells]


def extract_path(g, par, goal):
    w = g['w']
    cells = []
    c = int(goal[0] * w + goal[1])
    while c >= 0 and len(cells) < g['h'] * w:
        cells.append(divmod(c, w))
        c = int(par[c])
    cells.reverse()
    return [to_world(g, i, j) for i, j in cells]


def pick_frontier(g, dist, blacklist, hint, p):
    """Return (cell, n_clusters) of the best reachable frontier, or (None, 0)."""
    cs = g['cs']
    fr = g['frontier'].copy()
    for bx, by in blacklist:
        blank_around(g, fr, bx, by, p['blacklist_radius_m'])
    fr &= np.isfinite(dist)
    if not fr.any():
        return None, 0
    lab, n = ndimage.label(fr, structure=np.ones((3, 3)))
    idx = np.arange(1, n + 1)
    sizes = np.asarray(ndimage.sum(fr, lab, idx))
    keep = idx[sizes >= p['min_frontier_cells']]
    if keep.size == 0:
        return None, 0
    dmin = np.asarray(ndimage.minimum(dist, lab, keep))
    pos = ndimage.minimum_position(dist, lab, keep)
    score = dmin - 0.1 * sizes[keep - 1]          # mild preference for big frontiers
    choice = int(np.argmin(score))
    if hint is not None:                          # stick with the current goal (no dithering)
        hi, hj = to_cell(g, *hint)
        near = [k for k, c in enumerate(pos) if math.hypot(c[0] - hi, c[1] - hj) <= 8]
        if near:
            choice = min(near, key=lambda k: dmin[k])
    return tuple(int(v) for v in pos[choice]), len(keep)


def unvisited_targets(g, dist, trail, blacklist, p):
    """Centre cell of every reachable area the drone has not been near yet."""
    cs = g['cs']
    marks = np.zeros((g['h'], g['w']), bool)
    for x, y in trail:
        i, j = to_cell(g, x, y)
        if in_bounds(g, i, j):
            marks[i, j] = True
    if marks.any():
        visited = ndimage.distance_transform_edt(~marks) * cs <= p['visit_radius_m']
    else:
        visited = marks
    todo = g['trav'] & ~visited & np.isfinite(dist)
    for bx, by in blacklist:
        blank_around(g, todo, bx, by, p['blacklist_radius_m'])
    lab, n = ndimage.label(todo, structure=np.ones((3, 3)))
    deep = ndimage.distance_transform_edt(~visited) if visited.any() else None
    cells = []
    for k in range(1, n + 1):
        ii, jj = np.nonzero(lab == k)
        if len(ii) < p['min_sweep_cells']:
            continue
        if deep is not None:
            m = int(np.argmax(deep[ii, jj]))                   # deepest point of the area: stable target
        else:
            ci, cj = ii.mean(), jj.mean()
            m = int(np.argmin((ii - ci) ** 2 + (jj - cj) ** 2))
        cells.append((int(ii[m]), int(jj[m])))
    return cells


def detect_rooms(g, p, known, hub=None):
    """Find rooms = parts of the known free space joined to the rest only through a doorway that is
    narrower than 2*erode. Done at two erosion sizes (room_erode_m, room_erode2_m) so that both narrow
    doors (~1 m) and wide doors (~1.5 m) separate a room from the corridor network.
    The biggest part at each size is the corridor network; every other part is a room.
    The corridor network ('hub') is the part right behind the entrance (`hub` = a world point there);
    only if that is unknown, the biggest part is used.
    `known` = room points registered earlier: a part that already contains one is the same room.
    Returns the world position of the deepest point of each NEW room."""
    free = ~g['occ'] & ~g['unk']
    d = ndimage.distance_transform_edt(free) * g['cs']
    out = []
    for r in (p['room_erode_m'], p['room_erode2_m']):
        if r <= 0:
            continue
        core = free & (d >= r) & g['trav']
        lab, n = ndimage.label(core, structure=np.ones((3, 3)))
        if n < 2:
            continue
        sizes = np.asarray(ndimage.sum(core, lab, np.arange(1, n + 1)))
        corridor = int(np.argmax(sizes)) + 1
        if hub is not None:
            ci_, cj_ = np.nonzero(lab > 0)
            wx_, wy_ = cells_world(g, ci_, cj_)
            dd_ = np.hypot(wx_ - hub[0], wy_ - hub[1])
            m_ = int(np.argmin(dd_))
            if dd_[m_] <= 2.0:
                corridor = int(lab[ci_[m_], cj_[m_]])
        for k in range(1, n + 1):
            if k == corridor or sizes[k - 1] < p['room_min_cells']:
                continue
            same = False
            for q in list(known) + out:
                ci, cj = to_cell(g, *q)
                win = lab[max(0, ci - 1):ci + 2, max(0, cj - 1):cj + 2]
                if (win == k).any():
                    same = True
                    break
            if same:
                continue
            ii, jj = np.nonzero(lab == k)
            m = int(np.argmax(d[ii, jj]))
            pt = to_world(g, int(ii[m]), int(jj[m]))
            if any(math.hypot(pt[0] - q[0], pt[1] - q[1]) <= 1.0 for q in list(known) + out):
                continue
            out.append(pt)
    return out


def plan_tour(g, start, targets, exit_cell):
    """Order `targets` as a short open tour: start -> ... -> exit (nearest neighbour + 2-opt)."""
    nodes = [start] + list(targets)
    K = len(targets)
    dm, _ = dijkstra_multi(g, nodes)                 # one C-speed call for all nodes
    D = np.full((K + 1, K + 1), 1e9)
    ex = np.full(K + 1, 1e9)
    for a in range(K + 1):
        for b, c2 in enumerate(nodes):
            D[a][b] = dm[a][c2]
        ex[a] = dm[a][exit_cell]
    order, cur, left = [], 0, set(range(1, K + 1))
    while left:
        nxt = min(left, key=lambda j: D[cur][j])
        order.append(nxt); left.remove(nxt); cur = nxt

    def length(o):
        seq = [0] + o
        return sum(D[seq[i]][seq[i + 1]] for i in range(len(o))) + ex[seq[-1]]

    best, improved = length(order), True
    while improved and K > 2:
        improved = False
        for i in range(K - 1):
            for j in range(i + 1, K):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                L = length(cand)
                if L < best - 1e-6:
                    order, best, improved = cand, L, True
    return [targets[k - 1] for k in order]
