#!/usr/bin/env python3
"""/web_plan/map_show is built and sent only when the shown grid changes (T39).
node_planner/grid_cache.py + the two publishers of nodes/node_planner run
against it. No ROS master. Run: python3 test/test_grid_cache.py"""
import os
import re
import sys
import time
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from node_planner import grid_cache as gc  # noqa: E402

NODE = os.path.join(HERE, '..', 'nodes', 'node_planner')
H, W = 1035, 1931            # the live map of 16. 11.


def _info(x=0.0, res=0.05):
    o = types.SimpleNamespace(
        position=types.SimpleNamespace(x=x, y=2.0, z=0.0),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    return types.SimpleNamespace(resolution=res, origin=o, width=0, height=0)


class Cache(unittest.TestCase):
    def setUp(self):
        self.a = np.zeros((H, W), dtype=np.int8)
        self.c = gc.GridCache()

    def test_first_is_changed_then_quiet(self):
        self.assertTrue(self.c.changed(self.a, 'g'))
        self.c.sent()
        for _ in range(20):
            self.assertFalse(self.c.changed(self.a, 'g'))

    def test_not_sent_stays_changed(self):
        self.assertTrue(self.c.changed(self.a))
        self.assertTrue(self.c.changed(self.a))      # publish failed: try again

    def test_in_place_edit_of_one_cell_is_seen(self):
        self.c.changed(self.a); self.c.sent()
        self.a[H - 1, W - 1] = 100
        self.assertTrue(self.c.changed(self.a))

    def test_other_array_same_content_is_not_a_change(self):
        self.c.changed(self.a); self.c.sent()
        self.assertFalse(self.c.changed(self.a.copy()))

    def test_layer_swap_shape_and_geometry_are_seen(self):
        self.c.changed(self.a, gc.info_geometry(_info())); self.c.sent()
        b = self.a.copy(); b[3, 3] = -1
        self.assertTrue(self.c.changed(b, gc.info_geometry(_info())))
        self.assertTrue(self.c.changed(np.zeros((W, H), dtype=np.int8), gc.info_geometry(_info())))
        self.assertTrue(self.c.changed(self.a, gc.info_geometry(_info(x=1.0))))
        self.assertTrue(self.c.changed(self.a, gc.info_geometry(_info(res=0.1))))
        self.assertFalse(self.c.changed(self.a, gc.info_geometry(_info())))

    def test_invalidate_forces_one_send(self):
        self.c.changed(self.a); self.c.sent()
        self.c.invalidate()
        self.assertTrue(self.c.changed(self.a))
        self.c.sent()
        self.assertFalse(self.c.changed(self.a))

    def test_cells_built_once_and_equal_to_tolist(self):
        self.a[5, 7] = 100; self.a[9, 1] = -1
        want = self.a.reshape((self.a.size,)).tolist()
        got = self.c.cells(self.a)
        self.assertEqual(got, want)
        self.assertTrue(all(type(v) is int for v in got[:50]))
        for _ in range(10):
            self.assertIs(self.c.cells(self.a), got)
        self.assertEqual(self.c.builds, 1)
        self.a[0, 0] = 50
        self.assertEqual(self.c.cells(self.a)[0], 50)
        self.assertEqual(self.c.builds, 2)

    def test_non_contiguous_view(self):
        v = np.arange(48, dtype=np.int8).reshape((6, 8))[:, ::2]
        self.assertEqual(self.c.cells(v), v.reshape((v.size,)).tolist())
        self.assertTrue(self.c.changed(v))

    def test_idle_check_is_cheap(self):
        """The 2 Hz idle tick: a checksum, not a 2-million-cell list."""
        self.c.changed(self.a); self.c.sent()
        t0 = time.time()
        for _ in range(10):
            self.c.changed(self.a)
        per = (time.time() - t0) / 10
        t0 = time.time()
        self.a.reshape((self.a.size,)).tolist()
        self.assertLess(per, (time.time() - t0) / 3.0)


def _node_methods(*names):
    """The real publisher methods cut out of nodes/node_planner."""
    src = open(NODE).read()
    out = []
    for n in names:
        m = re.search(r'^    def %s\(self.*?(?=^    def |\Z)' % n, src, re.S | re.M)
        assert m, n
        out.append(m.group(0))
    return 'class Cut(object):\n' + '\n'.join(out)


class _Grid(object):
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.info = _info()
        self.data = []


class _Pub(object):
    made = []

    def __init__(self, *a, **k):
        self.msgs = []
        self.args, self.kwargs = a, k
        self.unregistered = False
        _Pub.made.append(self)

    def publish(self, m):
        assert not self.unregistered
        self.msgs.append(m)

    def unregister(self):
        self.unregistered = True


class NodePublishers(unittest.TestCase):
    def setUp(self):
        ns = {'OccupancyGrid': _Grid, 'grid_cache': gc, 'np': np,
              'rospy': types.SimpleNamespace(
                  Time=types.SimpleNamespace(now=lambda: 1.0), Publisher=_Pub,
                  logwarn=lambda *a, **k: None,
                  get_caller_id=lambda: 't', loginfo=lambda *a, **k: None)}
        exec(_node_methods('map_show_publisher', 'map_edited_publisher', 'callback_republish'), ns)
        del _Pub.made[:]
        n = ns['Cut']()
        n.np_show = np.zeros((40, 60), dtype=np.int8)
        n.map_data = types.SimpleNamespace(initial_map=types.SimpleNamespace(info=_info()),
                                           np_assembled=n.np_show)
        n.current_navi_map_name = 'LADA'
        n.map_show_pub, n.map_edited_pub = _Pub(), _Pub()
        n._grid_cache = gc.GridCache()
        n._edited_on_change = False                 # the default
        n._edited_state = gc.SentState()
        for f in ('map_data_publisher', 'map_poly_list_publisher',
                  'map_zone_list_publisher', 'program_list_publisher'):
            setattr(n, f, lambda: None)
        self.n = n

    def test_latched_in_the_node(self):
        src = open(NODE).read()
        self.assertEqual(len(re.findall(
            r'Publisher\("/web_plan/map_show", OccupancyGrid, latch=True', src)), 2)
        self.assertNotIn('Publisher("/web_plan/map_show", OccupancyGrid, queue_size', src)

    def test_map_show_once_per_change(self):
        n = self.n
        for _ in range(50):
            n.map_show_publisher()
        self.assertEqual(len(n.map_show_pub.msgs), 1)
        self.assertEqual(n._grid_cache.builds, 1)
        m = n.map_show_pub.msgs[0]
        self.assertEqual((m.info.width, m.info.height, len(m.data)), (60, 40, 2400))
        n.np_show[1, 2] = 100                       # edit in place
        for _ in range(5):
            n.map_show_publisher()
        self.assertEqual(len(n.map_show_pub.msgs), 2)
        self.assertEqual(n.map_show_pub.msgs[1].data[62], 100)
        n.np_show = np.full((40, 60), -1, dtype=np.int8)   # another layer
        n.map_show_publisher(); n.map_show_publisher()
        self.assertEqual(len(n.map_show_pub.msgs), 3)

    def test_no_map_no_message(self):
        self.n.map_data = None
        self.n.map_show_publisher()
        self.n.map_data, self.n.np_show = object(), None
        self.n.map_show_publisher()
        self.assertEqual(self.n.map_show_pub.msgs, [])

    def test_map_gone_drops_the_latch_once(self):
        """No map -> the latch must not keep serving the previous site's map:
        the publisher is advertised anew (empty latch), exactly once."""
        n = self.n
        n.map_show_publisher()
        old, keep = n.map_show_pub, n.map_data
        self.assertEqual(len(old.msgs), 1)
        n.map_data = None
        for _ in range(10):
            n.map_show_publisher()
        new = n.map_show_pub
        self.assertIsNot(new, old)
        self.assertTrue(old.unregistered)
        self.assertEqual(new.args[0], '/web_plan/map_show')
        self.assertTrue(new.kwargs.get('latch'))
        self.assertEqual(new.msgs, [])
        self.assertFalse(new.unregistered)
        self.assertEqual(len([p for p in _Pub.made if p.args[:1] == ('/web_plan/map_show',)]), 1)
        n.map_data = keep                           # the same map comes back
        n.map_show_publisher(); n.map_show_publisher()
        self.assertEqual(len(new.msgs), 1)

    def test_never_sent_no_map_keeps_the_publisher(self):
        n = self.n
        pub, n.map_data = n.map_show_pub, None
        n.map_show_publisher()
        self.assertIs(n.map_show_pub, pub)

    def test_republish_request_sends_again(self):
        n = self.n
        n.map_show_publisher()
        n.callback_republish(types.SimpleNamespace(data=True))
        n.map_show_publisher(); n.map_show_publisher()
        self.assertEqual(len(n.map_show_pub.msgs), 2)
        self.assertEqual(n._grid_cache.builds, 1)   # same grid: list reused

    def test_map_edited_keeps_its_cadence_but_builds_once(self):
        n = self.n
        n.map_show_publisher()
        for _ in range(8):
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 8)      # navi_man fed as before
        self.assertEqual(n._grid_cache.builds, 1)
        self.assertEqual(n.map_edited_pub.msgs[-1].data, n.np_show.reshape((2400,)).tolist())
        n.np_show[0, 0] = 100
        n.map_edited_publisher()
        self.assertEqual(n.map_edited_pub.msgs[-1].data[0], 100)
        n.current_navi_map_name = 'New'
        n.map_edited_publisher()
        self.assertEqual(n.map_edited_pub.msgs[-1].data, [])

    # ---- ~map_edited_on_change=true (T39 batch 3) ----
    def _on_change(self, refresh_s=0.0, clock=time.monotonic):
        self.n._edited_on_change = True
        self.n._edited_state = gc.SentState(refresh_s=refresh_s, clock=clock)
        return self.n

    def test_default_is_off_and_not_latched(self):
        src = open(NODE).read()
        self.assertIn("rospy.get_param('~map_edited_on_change', False)", src)
        self.assertIn("rospy.get_param('~map_edited_refresh_s', 0.0)", src)
        self.assertIn('latch=self._edited_on_change', src)

    def test_edited_on_change_sends_once_per_change(self):
        n = self._on_change()
        for _ in range(30):
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 1)      # first tick: late navi_man gets it from the latch
        self.assertEqual(n._grid_cache.builds, 1)
        m = n.map_edited_pub.msgs[0]
        self.assertEqual((m.info.width, m.info.height, len(m.data)), (60, 40, 2400))
        n.np_show[3, 4] = 100                                # a no-go zone drawn in the editor
        n.map_edited_publisher()                             # the very next 2 s tick
        self.assertEqual(len(n.map_edited_pub.msgs), 2)
        self.assertEqual(n.map_edited_pub.msgs[1].data[3 * 60 + 4], 100)
        for _ in range(10):
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 2)
        n.np_show = np.full((40, 60), -1, dtype=np.int8)     # another layer / reassembled map
        n.map_data.np_assembled = n.np_show
        n.map_edited_publisher(); n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 3)

    def test_edited_on_change_geometry_and_size(self):
        n = self._on_change()
        n.map_edited_publisher()
        n.map_data.initial_map.info.origin.position.x = 7.0  # same cells, moved map
        n.map_edited_publisher(); n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 2)
        n.map_data.np_assembled = np.zeros((41, 60), dtype=np.int8)
        n.map_edited_publisher(); n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 3)
        self.assertEqual(n.map_edited_pub.msgs[-1].info.height, 41)

    def test_edited_on_change_empty_states_once_each(self):
        """'New' map and 'no map' put an empty grid on the wire, once per transition
        (the latch must not keep the previous site's grid)."""
        n = self._on_change()
        n.map_edited_publisher()
        n.current_navi_map_name = 'New'
        for _ in range(5):
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 2)
        self.assertEqual(n.map_edited_pub.msgs[-1].data, [])
        keep, n.map_data = n.map_data, None
        for _ in range(5):
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 3)      # 'none' differs from 'new'
        self.assertEqual(n.map_edited_pub.msgs[-1].info.resolution, 0.05)
        n.map_data, n.current_navi_map_name = keep, 'LADA'   # the same map is back
        n.map_edited_publisher(); n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 4)
        self.assertEqual(len(n.map_edited_pub.msgs[-1].data), 2400)

    def test_fresh_node_sends_on_first_tick(self):
        """A restarted node_planner has nothing in its latch: whatever the state,
        the first tick publishes (no map yet -> empty grid, then the grid)."""
        n = self._on_change()
        keep, n.map_data = n.map_data, None
        n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 1)
        n.map_data = keep
        n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 2)

    def test_edited_refresh_period(self):
        t = [100.0]
        n = self._on_change(refresh_s=30.0, clock=lambda: t[0])
        n.map_edited_publisher()
        for _ in range(14):                                  # 28 s of 2 s ticks
            t[0] += 2.0
            n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 1)
        t[0] += 2.0
        n.map_edited_publisher()
        self.assertEqual(len(n.map_edited_pub.msgs), 2)
        self.assertEqual(n._grid_cache.builds, 1)            # refresh reuses the list

    def test_idle_tick_cost_on_change(self):
        n = self._on_change()
        n.np_show = np.zeros((H, W), dtype=np.int8)
        n.map_data.np_assembled = n.np_show
        n.map_edited_publisher()
        t0 = time.time()
        for _ in range(10):
            n.map_edited_publisher()
        per = (time.time() - t0) / 10
        self.assertEqual(len(n.map_edited_pub.msgs), 1)
        self.assertLess(per, 0.02)                           # a checksum of 2 M cells
        print('idle map_edited tick: %.1f ms' % (per * 1000.0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
