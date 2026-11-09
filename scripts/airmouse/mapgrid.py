"""Map helpers: down-sample + inflate the SLAM map, cell<->world conversion, save map to disk."""
import math
import os

import numpy as np
from .deps import ndimage


def prepare(grid, p, gate=None):
    """Down-sample the SLAM map and derive occupied / unknown / traversable / cost."""
    F = int(p['cell_factor'])
    H, W = grid.data.shape
    h, w = H // F, W // F
    if h < 5 or w < 5 or grid.data.size < F * F:
        return None
    data = grid.data[:h * F, :w * F]
    occ_f = data >= 50
    if p.get('speck_cells', 0) > 1 and occ_f.any():          # SLAM speckle: tiny isolated 'walls' must not block doorways
        lab, n = ndimage.label(ndimage.binary_dilation(occ_f, structure=np.ones((3, 3))), structure=np.ones((3, 3)))
        cnt = np.asarray(ndimage.sum(occ_f, lab, np.arange(1, n + 1)))
        occ_f = occ_f & np.isin(lab, np.nonzero(cnt >= p['speck_cells'])[0] + 1)
        data = np.where(occ_f, data, np.where(data >= 50, 0, data)).astype(np.int8)
    b = data.reshape(h, F, w, F)
    occ = (b >= 50).sum(axis=(1, 3)) >= 2
    unk = ((b < 0).sum(axis=(1, 3)) > (F * F) // 2) & ~occ
    cs = grid.res * F
    # clearance from the FINE map (0.05 m), sampled at the centre of each coarse cell: accurate to ~0.05 m,
    # so a 0.78 m gap and a 1.0 m corridor can be told apart (the coarse 0.2 m map cannot)
    if occ_f.any():
        df = ndimage.distance_transform_edt(~occ_f) * grid.res
        d = df[np.ix_(np.arange(h) * F + F // 2, np.arange(w) * F + F // 2)]
    else:
        d = np.full(occ.shape, 1e3)
    trav = ~occ & ~unk & (d >= p['inflate_m'])
    cost = 1.0 + p['proximity_penalty'] * np.clip(
        (p['pref_clear_m'] - d) / p['pref_clear_m'], 0.0, 1.0)
    g = dict(occ=occ, unk=unk, trav=trav, cost=cost, d=d, cs=cs, ox=grid.ox, oy=grid.oy,
             cy=math.cos(grid.yaw), sy=math.sin(grid.yaw), h=h, w=w)
    if gate is not None:                      # forbid everything behind the gate line
        hx, hy, fx, fy, gm = gate
        ii, jj = np.mgrid[0:h, 0:w]
        cx, cy = cells_world(g, ii, jj)
        g['trav'] = trav & (((cx - hx) * fx + (cy - hy) * fy) >= gm)
    # frontier = traversable cell touching a REAL unknown region; tiny unknown specks (noise) don't count
    ulab, un = ndimage.label(unk, structure=np.ones((3, 3)))
    if un:
        usz = np.asarray(ndimage.sum(unk, ulab, np.arange(1, un + 1)))
        unk_big = np.isin(ulab, np.nonzero(usz >= p['min_unknown_cells'])[0] + 1)
    else:
        unk_big = unk
    g['frontier'] = g['trav'] & ndimage.binary_dilation(unk_big, structure=np.ones((3, 3)))
    return g


def to_cell(g, x, y):
    """World (x, y) -> coarse cell (row, col). floor() so negative / out-of-map points map to
    negative or too-large indices instead of silently landing in cell 0. Use in_bounds() before indexing."""
    dx, dy = x - g['ox'], y - g['oy']
    lx = g['cy'] * dx + g['sy'] * dy          # rotate into the map-origin frame
    ly = -g['sy'] * dx + g['cy'] * dy
    return int(math.floor(ly / g['cs'])), int(math.floor(lx / g['cs']))


def to_world(g, i, j):
    """Coarse cell centre -> world (x, y). Exact inverse of to_cell for cell centres."""
    lx, ly = (j + 0.5) * g['cs'], (i + 0.5) * g['cs']
    return (g['ox'] + g['cy'] * lx - g['sy'] * ly, g['oy'] + g['sy'] * lx + g['cy'] * ly)


def cells_world(g, ii, jj):
    """Vectorised to_world for index arrays."""
    lx, ly = (jj + 0.5) * g['cs'], (ii + 0.5) * g['cs']
    return (g['ox'] + g['cy'] * lx - g['sy'] * ly, g['oy'] + g['sy'] * lx + g['cy'] * ly)


def in_bounds(g, i, j):
    return 0 <= i < g['h'] and 0 <= j < g['w']


def blank_around(g, arr, x, y, radius_m):
    """Set arr[...] = False in a square of +-radius_m around world point (x, y); safe at the map edge."""
    ci, cj = to_cell(g, x, y)
    r = int(radius_m / g['cs']) + 1
    i0, i1, j0, j1 = max(0, ci - r), min(g['h'], ci + r + 1), max(0, cj - r), min(g['w'], cj + r + 1)
    if i1 > i0 and j1 > j0:
        arr[i0:i1, j0:j1] = False


def save_map(grid, prefix):
    H, W = grid.data.shape
    img = np.full((H, W), 205, np.uint8)                     # unknown
    img[(grid.data >= 0) & (grid.data < 25)] = 254           # free
    img[grid.data >= 65] = 0                                 # occupied
    img = np.flipud(img)
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(img.tobytes())
    with open(prefix + '.yaml', 'w') as f:
        f.write('image: %s\nmode: trinary\nresolution: %.4f\norigin: [%.4f, %.4f, %.4f]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                % (os.path.basename(prefix) + '.pgm', grid.res, grid.ox, grid.oy, grid.yaw))
