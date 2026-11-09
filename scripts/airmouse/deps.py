"""Optional libraries, imported once with a friendly error message."""
import sys

import numpy as np

try:
    from scipy import ndimage
except Exception as _e:                # typical on Ubuntu 24.04: apt SciPy built for NumPy 1.x + pip NumPy 2.x
    sys.stderr.write(
        '\nCannot import scipy.ndimage (%s).\nNumPy %s is installed. Fix (one line):\n'
        '    pip3 install --user --upgrade scipy --break-system-packages\n\n' % (_e, np.__version__))
    raise
try:                                   # C-speed shortest paths; falls back to pure Python if unavailable
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra as _sp_dijkstra
except Exception:                      # pragma: no cover
    csr_matrix = None
    _sp_dijkstra = None
