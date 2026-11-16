"""Publish a big grid only when it changed (T39).

node_planner used to turn the shown map (about 2 million cells) into a Python
list twice a second for /web_plan/map_show, a topic that usually has no
subscriber at all - 12 % of a core for nothing. The grid changes only when the
map is edited or another layer is picked, so a cheap checksum of the cells
decides whether anything has to be built.

No ROS in here: the node hands in the numpy array and the map geometry.
"""
import zlib

import numpy as np


def fingerprint(arr, geometry=None):
    """Cheap identity of a grid's CONTENT (not of the array object): the layers
    are edited in place as well as swapped, so the object id says nothing.
    ~2-3 ms for 2 million cells."""
    buf = np.ascontiguousarray(arr)
    return (buf.shape, buf.dtype.str, zlib.crc32(buf), zlib.adler32(buf), geometry)


def info_geometry(info):
    """The fields of nav_msgs/MapMetaData that place the grid in the world."""
    p, q = info.origin.position, info.origin.orientation
    return (info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class GridCache(object):
    """Remembers the last grid turned into a flat list and what was last sent.

    cells(arr)        -> flat list of the grid; rebuilt only when the content
                         changed (shared by every topic showing the same grid).
    changed(arr, geo) -> True when this grid/geometry differs from the one last
                         confirmed by sent(); the caller then publishes.
    sent()            -> confirm the publish of what changed() just looked at.
    invalidate()      -> force the next changed() to be True.
    has_sent()        -> something was sent and not invalidated since.
    """

    def __init__(self):
        self._cells_fp = None
        self._cells = None
        self._sent_fp = None
        self._pending_fp = None
        self.builds = 0          # how many times the list was really built

    def cells(self, arr):
        fp = fingerprint(arr)
        if fp != self._cells_fp or self._cells is None:
            self._cells = np.ascontiguousarray(arr).reshape((arr.size,)).tolist()
            self._cells_fp = fp
            self.builds += 1
        return self._cells

    def changed(self, arr, geometry=None):
        self._pending_fp = fingerprint(arr, geometry)
        return self._pending_fp != self._sent_fp

    def sent(self):
        self._sent_fp = self._pending_fp

    def invalidate(self):
        self._sent_fp = None

    def has_sent(self):
        """True while something confirmed by sent() is still considered out
        there (i.e. a latched publisher holds it)."""
        return self._sent_fp is not None
