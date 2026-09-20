#!/usr/bin/env python3
"""
planner_io.py -- pure helpers for node_planner (NO rospy / ROS-message imports)

Kept ROS-free so the logic is unit-testable offline (see test/test_planner_io.py):

  * navi_pkg_path()        -- locate the vitulus_navi package without depending
                              on the name/location of the catkin workspace.
  * save_pickle_checked()  -- never persist a None workspace (a pickled None is
                              a valid 4-byte file that later "loads" as nothing).
  * load_pickle_checked()  -- load + validate; a missing / None / foreign pickle
                              is reported as "no saved data", not as a crash.
  * write_json_atomic()    -- tmp + os.replace, for small state markers.
  * merge_zone_names()     -- programs.yaml export rule: a program never loses
                              a zone only because the workspace could not
                              regenerate it right now.
"""

import json
import os
import pickle

# Historical location on the reference robot. Last-resort fallback only, so the
# behaviour on the standard layout stays identical when rospkg is unavailable.
_LEGACY_NAVI_PATH = "/home/vitulus/catkin_ws/src/vitulus/vitulus_navi"


def navi_pkg_path(rospack_get_path=None, here=None, legacy=_LEGACY_NAVI_PATH):
    """Absolute path of the vitulus_navi package.

    Resolution order:
      1. rospkg (authoritative -- whatever workspace is sourced),
      2. a 'vitulus_navi' checkout next to this package (same src/ folder),
      3. the historical absolute path.
    `rospack_get_path` / `here` are injectable for the offline test."""
    if rospack_get_path is None:
        try:
            import rospkg
            rospack_get_path = rospkg.RosPack().get_path
        except Exception:
            rospack_get_path = None
    if rospack_get_path is not None:
        try:
            path = rospack_get_path('vitulus_navi')
            if path and os.path.isdir(path):
                return path
        except Exception:
            pass
    # <src>/vitulus_planner/src/node_planner/planner_io.py -> <src>/vitulus_navi
    here = here or os.path.realpath(__file__)
    d = os.path.dirname(here)
    for _ in range(6):
        cand = os.path.join(d, 'vitulus_navi')
        if os.path.isfile(os.path.join(cand, 'package.xml')):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return legacy


def save_pickle_checked(path, obj):
    """Pickle `obj` to `path` atomically. Returns False (and writes NOTHING)
    when obj is None; raises on I/O errors."""
    if obj is None:
        return False
    tmp = path + '.tmp'
    with open(tmp, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)
    os.replace(tmp, path)
    return True


def load_pickle_checked(path, required_attrs=()):
    """Load a pickle and validate it. Returns (obj, reason):
      (obj, None)          -- loaded and valid,
      (None, 'missing')    -- file does not exist,
      (None, 'empty')      -- file holds a pickled None (or is zero bytes),
      (None, 'invalid: …') -- unreadable, or lacks one of `required_attrs`.
    Never raises."""
    if not os.path.exists(path):
        return None, 'missing'
    try:
        if os.path.getsize(path) == 0:
            return None, 'empty'
        with open(path, 'rb') as f:
            obj = pickle.load(f)
    except Exception as e:
        return None, 'invalid: {}'.format(e)
    if obj is None:
        return None, 'empty'
    for attr in required_attrs:
        if not hasattr(obj, attr):
            return None, 'invalid: {} has no attribute {!r}'.format(
                type(obj).__name__, attr)
    return obj, None


def write_json_atomic(path, obj):
    """Write `obj` as JSON via tmp + os.replace (a crash never leaves a
    half-written marker). Raises on I/O errors."""
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def merge_zone_names(prev_names, mem_names, workspace_names, bundle_zone_names):
    """zone_names to write into programs.yaml for ONE program.

    prev_names        -- zone_names currently stored in programs.yaml
    mem_names         -- zone names of the program in memory (mowing order)
    workspace_names   -- zone names present in the planner workspace, or None
                         when the workspace of that site is not available
    bundle_zone_names -- zone names stored in the site's zones.geojson

    Same idea as the zones.geojson merge export: a stored zone name that the
    program lost ONLY because the workspace does not hold the zone (raster not
    built, zone outside the current raster coverage, datum missing) is KEPT as
    long as the zone still exists in zones.geojson. A zone that IS in the
    workspace but no longer in the program was removed on purpose -> dropped.
    The stored order is preserved when the memory order is compatible with it;
    otherwise the memory order wins and the kept names are appended."""
    prev = [n for n in (prev_names or [])]
    mem = [n for n in (mem_names or [])]
    ws = None if workspace_names is None else set(workspace_names)
    bundle = set(bundle_zone_names or [])
    kept = [n for n in prev
            if n not in mem and n in bundle and (ws is None or n not in ws)]
    if not kept:
        return mem
    wanted = set(mem) | set(kept)
    in_prev_order = [n for n in prev if n in wanted]
    # memory order compatible with the stored order (and nothing new)?
    if [n for n in in_prev_order if n in mem] == mem:
        return in_prev_order
    return mem + kept
