#!/usr/bin/env python3
"""Offline tests for node_planner.planner_io (no ROS master, no rospy).

Run:  python3 test/test_planner_io.py
"""
import os
import pickle
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'node_planner'))
import planner_io  # noqa: E402  (imported as a plain module: no ROS deps)


class Workspace(object):
    """Stand-in for MapData (module-level so it can be pickled)."""
    def __init__(self):
        self.zones = []
        self.resolution = 0.05


class PickleTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, 'planner_data.pkl')

    def test_none_is_never_saved(self):
        self.assertFalse(planner_io.save_pickle_checked(self.path, None))
        self.assertFalse(os.path.exists(self.path))

    def test_roundtrip(self):
        self.assertTrue(planner_io.save_pickle_checked(self.path, Workspace()))
        self.assertFalse(os.path.exists(self.path + '.tmp'))
        obj, reason = planner_io.load_pickle_checked(
            self.path, required_attrs=('zones', 'resolution'))
        self.assertIsNone(reason)
        self.assertEqual(obj.resolution, 0.05)

    def test_missing(self):
        self.assertEqual(planner_io.load_pickle_checked(self.path),
                         (None, 'missing'))

    def test_legacy_pickled_none_is_no_data(self):
        # The 4-byte file older planners wrote with map_data == None.
        with open(self.path, 'wb') as f:
            pickle.dump(None, f, pickle.HIGHEST_PROTOCOL)
        self.assertEqual(planner_io.load_pickle_checked(self.path, ('zones',)),
                         (None, 'empty'))

    def test_zero_bytes_and_garbage(self):
        open(self.path, 'wb').close()
        self.assertEqual(planner_io.load_pickle_checked(self.path),
                         (None, 'empty'))
        with open(self.path, 'wb') as f:
            f.write(b'not a pickle')
        obj, reason = planner_io.load_pickle_checked(self.path)
        self.assertIsNone(obj)
        self.assertTrue(reason.startswith('invalid'))

    def test_foreign_object_is_invalid(self):
        planner_io.save_pickle_checked(self.path, {'zones': []})
        obj, reason = planner_io.load_pickle_checked(self.path, ('zones',))
        self.assertIsNone(obj)
        self.assertTrue(reason.startswith('invalid'))


class NaviPathTest(unittest.TestCase):
    def test_rospack_wins(self):
        d = tempfile.mkdtemp()
        self.assertEqual(planner_io.navi_pkg_path(rospack_get_path=lambda n: d), d)

    def test_sibling_checkout_fallback(self):
        # <ws>/any_name/src/{vitulus_navi,vitulus_planner/src/node_planner}
        ws = tempfile.mkdtemp()
        navi = os.path.join(ws, 'vitulus_navi')
        here = os.path.join(ws, 'vitulus_planner', 'src', 'node_planner')
        os.makedirs(navi)
        os.makedirs(here)
        open(os.path.join(navi, 'package.xml'), 'w').close()

        def boom(name):
            raise RuntimeError('package not found')
        self.assertEqual(
            planner_io.navi_pkg_path(rospack_get_path=boom,
                                     here=os.path.join(here, 'planner_io.py')),
            navi)

    def test_legacy_last_resort(self):
        def boom(name):
            raise RuntimeError('package not found')
        self.assertEqual(
            planner_io.navi_pkg_path(rospack_get_path=boom,
                                     here='/nonexistent/a/b/planner_io.py',
                                     legacy='/legacy/vitulus_navi'),
            '/legacy/vitulus_navi')


class MergeZoneNamesTest(unittest.TestCase):
    merge = staticmethod(planner_io.merge_zone_names)

    def test_workspace_lost_zone_is_kept_in_stored_order(self):
        # 'Far' exists in zones.geojson but the workspace could not build it
        self.assertEqual(self.merge(['Near', 'Far', 'Back'], ['Near', 'Back'],
                                    ['Near', 'Back'], ['Near', 'Far', 'Back']),
                         ['Near', 'Far', 'Back'])

    def test_workspace_unknown_keeps_every_bundle_zone(self):
        self.assertEqual(self.merge(['Near', 'Far'], [], None, ['Near', 'Far']),
                         ['Near', 'Far'])

    def test_zone_removed_from_program_on_purpose_is_dropped(self):
        # 'Far' IS in the workspace, the operator took it out of the program
        self.assertEqual(self.merge(['Near', 'Far'], ['Near'],
                                    ['Near', 'Far'], ['Near', 'Far']), ['Near'])

    def test_deleted_zone_is_dropped(self):
        # 'Far' is gone from zones.geojson
        self.assertEqual(self.merge(['Near', 'Far'], ['Near'], ['Near'], ['Near']),
                         ['Near'])

    def test_memory_order_wins_on_reorder_or_new_zone(self):
        self.assertEqual(self.merge(['A', 'B', 'C'], ['C', 'A', 'New'],
                                    ['A', 'C', 'New'], ['A', 'B', 'C', 'New']),
                         ['C', 'A', 'New', 'B'])

    def test_no_previous_entry(self):
        self.assertEqual(self.merge(None, ['A'], None, []), ['A'])


class JsonAtomicTest(unittest.TestCase):
    def test_write(self):
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'marker.json')
        planner_io.write_json_atomic(path, {'site': 'A'})
        planner_io.write_json_atomic(path, {'site': 'B'})
        import json
        with open(path) as f:
            self.assertEqual(json.load(f), {'site': 'B'})
        self.assertEqual(os.listdir(d), ['marker.json'])


if __name__ == '__main__':
    unittest.main()
