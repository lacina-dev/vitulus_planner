#!/usr/bin/env python3
"""Map rules in coverage, the zone dry run and zone split/merge/duplicate
(docs: vitulus_mapping docs/MAP_OBJECTS.md, Coverage and section 6).

No ROS master: the same stubbed node and temporary $HOME as test_site_programs.
The first class needs neither (coverage_rules is pure).

Run:  python3 test/test_zone_rules.py
"""
import json
import os
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), 'src'))
import test_site_programs as T  # noqa: E402  (sets the temporary $HOME first)

import numpy as np  # noqa: E402
from shapely import geometry  # noqa: E402
from node_planner import coverage_rules as cr  # noqa: E402

SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]


def rule(kind, x0, y0, x1, y1, name='r'):
    return {'id': 'o_1', 'type': kind, 'name': name, 'speed_mps': None,
            'points': [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]}


def rules_json(rules, site='A', seq=1):
    return json.dumps({'site': site, 'seq': seq, 'stamp': seq, 'rules': rules})


class PureTest(unittest.TestCase):

    def test_parse_keeps_only_what_shapes_coverage(self):
        site, rules = cr.parse_rules(rules_json([
            rule('nogo', 0, 0, 1, 1), rule('noblade', 0, 0, 1, 1), rule('slow', 0, 0, 1, 1),
            {'type': 'nogo', 'points': [[0, 0], [1, 1]]}]))
        self.assertEqual(site, 'A')
        self.assertEqual([r['type'] for r in rules], ['nogo', 'noblade'])
        with self.assertRaises(ValueError):
            cr.parse_rules('[]')

    def test_fingerprint_is_the_set_not_the_message(self):
        a = cr.parse_rules(rules_json([rule('nogo', 0, 0, 1, 1), rule('noblade', 2, 2, 3, 3)]))[1]
        b = cr.parse_rules(rules_json([rule('noblade', 2, 2, 3, 3, name='x'),
                                       rule('nogo', 0, 0, 1, 1)], seq=9))[1]
        c = cr.parse_rules(rules_json([rule('nogo', 0, 0, 1, 1.5)]))[1]
        d = cr.parse_rules(rules_json([rule('noblade', 0, 0, 1, 1)]))[1]
        e = cr.parse_rules(rules_json([rule('nogo', 0, 0, 1, 1)]))[1]
        self.assertEqual(cr.fingerprint(a), cr.fingerprint(b))
        self.assertNotEqual(cr.fingerprint(c), cr.fingerprint(e))
        self.assertNotEqual(cr.fingerprint(d), cr.fingerprint(e))
        self.assertEqual(cr.fingerprint([]), '')

    def test_rules_of_zone_by_box(self):
        rules = cr.parse_rules(rules_json([rule('nogo', 5, 5, 6, 6, 'in'),
                                           rule('nogo', 10.5, 0, 11, 1, 'near'),
                                           rule('nogo', 30, 30, 31, 31, 'far')]))[1]
        self.assertEqual([r['name'] for r in cr.rules_of_zone(rules, SQUARE)], ['in', 'near'])

    def test_stamp_grows_and_nogo_grows_more(self):
        rules = cr.parse_rules(rules_json([rule('nogo', 5, 5, 6, 6)]))[1]
        blade = cr.parse_rules(rules_json([rule('noblade', 5, 5, 6, 6)]))[1]
        px = lambda x, y: (int(x / 0.1), int(y / 0.1))
        a = cr.stamp(np.zeros((100, 100), dtype=np.int8), rules, px, 2, 3, 100)
        b = cr.stamp(np.zeros((100, 100), dtype=np.int8), blade, px, 2, 3, 100)
        self.assertEqual(a[55, 45], 100)     # 5 px out: margin + extra
        self.assertEqual(a[55, 44], 0)
        self.assertEqual(b[55, 48], 100)     # 2 px out: margin only
        self.assertEqual(b[55, 47], 0)

    def test_cut_height(self):
        for ok in (0, 20, 45, 80):
            self.assertTrue(cr.cut_height_ok(ok), ok)
        for bad in (-1, 1, 19, 81, 90):
            self.assertFalse(cr.cut_height_ok(bad), bad)

    def test_split(self):
        parts, code = cr.split_zone(SQUARE, [[3, -1], [3, 11]])
        self.assertIsNone(code)
        self.assertAlmostEqual(geometry.Polygon(parts[0]).area, 70)     # larger first
        self.assertAlmostEqual(geometry.Polygon(parts[1]).area, 30)

    def test_split_line_inside_the_zone_is_run_on(self):
        parts, code = cr.split_zone(SQUARE, [[5, 4], [5, 6]])
        self.assertIsNone(code)
        self.assertEqual(len(parts), 2)

    def test_split_refusals(self):
        self.assertEqual(cr.split_zone(SQUARE, [[20, 0], [20, 5]])[1], 'line_misses_zone')
        self.assertEqual(cr.split_zone(SQUARE, [[1, 1], [1, 1]])[1], 'bad_line')
        self.assertEqual(cr.split_zone(SQUARE, 'x')[1], 'bad_line')
        comb = [(0, 0), (10, 0), (10, 10), (7, 10), (7, 3), (3, 3), (3, 10), (0, 10)]
        self.assertEqual(cr.split_zone(comb, [[-1, 6], [11, 6]])[1], 'split_not_two_parts')

    def test_merge(self):
        a = [(0, 0), (5, 0), (5, 5), (0, 5)]
        b = [(5, 0), (9, 0), (9, 5), (5, 5)]
        shape, code = cr.merge_zones([a, b])
        self.assertIsNone(code)
        self.assertAlmostEqual(geometry.Polygon(shape).area, 45, places=3)
        self.assertEqual(len(shape), 4)      # the seam is gone

    def test_merge_small_gap_is_adjacent(self):
        a = [(0, 0), (5, 0), (5, 5), (0, 5)]
        b = [(5.05, 0), (9, 0), (9, 5), (5.05, 5)]
        self.assertIsNone(cr.merge_zones([a, b])[1])

    def test_merge_refusals(self):
        a = [(0, 0), (5, 0), (5, 5), (0, 5)]
        far = [(8, 0), (9, 0), (9, 5), (8, 5)]
        self.assertEqual(cr.merge_zones([a, far])[1], 'not_adjacent')
        ring = [[(0, 0), (9, 0), (9, 3), (0, 3)], [(0, 6), (9, 6), (9, 9), (0, 9)],
                [(0, 3), (3, 3), (3, 6), (0, 6)], [(6, 3), (9, 3), (9, 6), (6, 6)]]
        self.assertEqual(cr.merge_zones(ring)[1], 'merge_has_hole')

    def test_unique_name(self):
        self.assertEqual(cr.unique_name('A copy', ['A']), 'A copy')
        self.assertEqual(cr.unique_name('A copy', ['a COPY', 'A copy 2']), 'A copy 3')

    def test_valid_parts(self):
        good = geometry.box(0, 0, 1, 1)
        self.assertEqual(cr.valid_parts(good), [good])
        hole_on_edge = geometry.Polygon(
            [(0, 0), (4, 0), (4, 4), (0, 4)],
            holes=[[(0, 1), (1, 1), (1, 3), (0, 3)], [(0, 1), (0.5, 1.5), (0, 2)]])
        parts = cr.valid_parts(hole_on_edge)
        self.assertTrue(parts)
        self.assertTrue(all(p.is_valid for p in parts))

    def test_read_shape(self):
        self.assertEqual(cr.read_shape(None)[2][0]['code'], 'bad_points')
        self.assertEqual(cr.read_shape([[0, 0], [1, 1]])[2][0]['code'], 'bad_points')
        bow = [[0, 0], [2, 2], [2, 0], [0, 2]]
        self.assertEqual(cr.read_shape(bow)[2][0]['code'], 'self_intersecting')
        self.assertEqual(cr.read_shape(SQUARE)[2], [])

    def test_minutes(self):
        runs = [{'length': 58, 'minutes': 9, 'speed': 'slow', 'zones': ['A'], 'result': 'succeeded'},
                {'length': 100, 'minutes': 50, 'speed': 'slow', 'zones': ['B'], 'result': 'succeeded'},
                {'length': 100, 'minutes': 1, 'speed': 'slow', 'zones': ['A'], 'result': 'failed: on_init'}]
        self.assertEqual(cr.estimate_minutes(58, 'slow', runs, 'A'), (9, 'history'))
        self.assertEqual(cr.estimate_minutes(158, 'slow', runs, 'new'), (59, 'history'))
        self.assertEqual(cr.estimate_minutes(100, 'fast', runs, 'A')[1], 'speed')
        self.assertEqual(cr.estimate_minutes(108, 'mid', [])[0], 10)
        self.assertEqual(cr.estimate_minutes(108, 'nonsense', [])[0], 10)

    def test_preview_is_capped_and_keeps_ends(self):
        line = [(i * 0.1, 0.0) for i in range(5000)] + [(499.9, 0.0)]
        out = cr.thin_preview([line, [(1, 1), (1, 1), (2, 2)]], max_points=100)
        self.assertLessEqual(sum(len(x) for x in out), 110)
        self.assertEqual(out[0][0], [0.0, 0.0])
        self.assertEqual(out[0][-1], [499.9, 0.0])
        self.assertEqual(out[1], [[1.0, 1.0], [2.0, 2.0]])

    def test_cut_off_parts(self):
        free = np.ones((100, 100), dtype=bool)
        free[:, 48:52] = False
        px = lambda x, y: (int(x / 0.1), int(y / 0.1))
        left = geometry.box(1, 1, 3, 9)
        right = geometry.box(6, 1, 9, 9)
        self.assertEqual(cr.cut_off_parts([left, right], free, px), [0])     # largest stays
        self.assertEqual(cr.cut_off_parts([left, right], free, px, home=(1, 5)), [1])
        free[50, 48:52] = True
        self.assertEqual(cr.cut_off_parts([left, right], free, px, home=(1, 5)), [])


@unittest.skipIf(T._SKIP is not None, T._SKIP)
class NodeCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        T._load_module()
        cls.M = T.M

    setUp = T.SiteProgramsTest.setUp

    def site(self):
        """Site A, zones Near and Far, program MowAll on both."""
        n = T.new_node()
        T.serve(n, 'A')
        n.callback_map_save_zone(T.zmsg('Near', 2, 2, 8, 8))
        n.callback_map_save_zone(T.zmsg('Far', 12, 12, 16, 16))
        n.callback_program_new(T.program('MowAll', T.zone_msgs(n, 'Near', 'Far')))
        return n

    def zone(self, n, name):
        return [z for z in n.map_data.zones if z.msg.name == name][0]

    def stored(self, n, zone='Near', program='MowAll'):
        p = [p for p in n.permanent_program_list_msg.program_list if p.name == program][0]
        return [z for z in p.zone_list if z.name == zone][0]

    def rules(self, n, *rules, **kw):
        n.callback_map_rules(T.String(rules_json(list(rules), **kw)))

    def run_worker(self, n):
        """What the debounce timer would start, synchronously."""
        n._site_map_worker()

    def count_regen(self, n):
        calls = []
        real = n._regenerate_zone

        def counted(msg):
            calls.append(msg.name)
            return real(msg)
        n._regenerate_zone = counted
        return calls

    @staticmethod
    def segments(zmsg):
        """Consecutive poses inside each Path: what the robot follows blade down."""
        for path in zmsg.paths:
            pts = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
            for a, b in zip(pts, pts[1:]):
                if a != b:
                    yield geometry.LineString([a, b])


class CoverageTest(NodeCase):

    def test_nogo_keeps_paths_away_and_navi_layer_as_it_was(self):
        n = self.site()
        navi = self.zone(n, 'Near').np_zone_navi.copy()
        length = self.zone(n, 'Near').msg.length
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        z = self.zone(n, 'Near')
        self.assertLess(z.msg.length, length)
        area = geometry.box(4, 4, 6, 6)
        for seg in self.segments(z.msg):
            self.assertGreaterEqual(seg.distance(area), 0.38)
        self.assertTrue((z.np_zone_navi == navi).all())

    def test_noblade_strip_splits_coverage_without_a_connector_across(self):
        n = self.site()
        navi = self.zone(n, 'Near').np_zone_navi.copy()
        self.rules(n, rule('noblade', 4.5, 0, 5.5, 10))
        self.run_worker(n)
        z = self.zone(n, 'Near')
        strip = geometry.box(4.5, 0, 5.5, 10)
        segs = list(self.segments(z.msg))
        self.assertGreater(len(segs), 20)
        for seg in segs:
            self.assertFalse(seg.intersects(strip), seg.wkt)
            self.assertGreaterEqual(seg.distance(strip), 0.33)
        # both sides are mowed, and each side in few paths: connectors inside a
        # part work although the part is one of two outline polygons
        xs = [q.pose.position.x for p in z.msg.paths for q in p.poses]
        self.assertLess(min(xs), 3.5)
        self.assertGreater(max(xs), 6.5)
        self.assertLessEqual(len(z.msg.paths), 10)
        self.assertTrue((z.np_zone_navi == navi).all())     # still passable

    def test_rule_over_the_whole_zone_leaves_no_path(self):
        n = self.site()
        self.rules(n, rule('nogo', 1, 1, 9, 9))
        self.run_worker(n)
        self.assertEqual(list(self.zone(n, 'Near').msg.paths), [])
        self.assertEqual(list(self.stored(n).paths), [])

    def test_failed_regen_never_keeps_paths_through_a_rule(self):
        n = self.site()
        self.assertTrue(self.zone(n, 'Near').msg.paths)
        real = n._regenerate_zone

        def boom(msg):
            if msg.name == 'Near':
                raise ValueError('boom')
            return real(msg)
        n._regenerate_zone = boom
        self.rules(n, rule('nogo', 4, 4, 6, 6), rule('nogo', 13, 13, 14, 14))
        self.run_worker(n)
        self.assertEqual(list(self.zone(n, 'Near').msg.paths), [])
        self.assertEqual(list(self.stored(n).paths), [])
        self.assertTrue(self.zone(n, 'Far').msg.paths)

    def test_odd_rule_positions_never_break_generation(self):
        n = self.site()
        rnd = np.random.RandomState(7)
        for i in range(25):
            x, y = rnd.uniform(1, 8, 2)
            w, h = rnd.uniform(0.1, 4, 2)
            self.rules(n, rule(('nogo', 'noblade')[i % 2], x, y, x + w, y + h), seq=i)
            before = T.LOGS[:]
            self.run_worker(n)
            bad = [m for lvl, m in T.LOGS[len(before):] if 'failed' in m]
            self.assertEqual(bad, [], (x, y, w, h))
            self.assertEqual(self.zone(n, 'Near').rules_fp != '', True)

    def test_save_applies_the_rules_too(self):
        n = self.site()
        self.rules(n, rule('nogo', 22, 22, 23, 23))
        calls = self.count_regen(n)
        self.run_worker(n)
        self.assertEqual(calls, [])
        n.callback_map_save_zone(T.zmsg('Other', 18, 18, 19.8, 19.8))
        self.assertEqual(self.zone(n, 'Other').rules_fp, '')
        self.rules(n, rule('nogo', 19, 19, 19.5, 19.5))
        self.assertEqual(self.count_regen(n), [])
        n.callback_map_save_zone(T.zmsg('Other', 18, 16, 19.8, 19.8))
        self.assertNotEqual(self.zone(n, 'Other').rules_fp, '')


class TriggerTest(NodeCase):

    def test_same_set_new_seq_regenerates_nothing(self):
        n = self.site()
        self.rules(n, rule('nogo', 4, 4, 6, 6), seq=1)
        self.run_worker(n)
        calls = self.count_regen(n)
        self.rules(n, rule('nogo', 4, 4, 6, 6), seq=2)
        self.rules(n, rule('nogo', 4, 4, 6, 6), rule('slow', 3, 3, 4, 4), seq=3)
        self.assertFalse(n._site_map_rules_dirty)
        self.run_worker(n)
        self.assertEqual(calls, [])

    def test_change_regenerates_only_the_zones_it_bears_on(self):
        n = self.site()
        calls = self.count_regen(n)
        self.rules(n, rule('noblade', 4, 4, 6, 6))
        self.assertTrue(n._site_map_rules_dirty)
        self.run_worker(n)
        self.assertEqual(calls, ['Near'])
        self.assertFalse(n._site_map_rules_dirty)

    def test_burst_is_one_run(self):
        n = self.site()
        calls = self.count_regen(n)
        for i in range(1, 6):
            self.rules(n, rule('noblade', 4, 4, 4 + i, 6), seq=i)
        self.run_worker(n)
        self.assertEqual(calls, ['Near'])

    def test_removing_the_rule_restores_the_paths(self):
        n = self.site()
        length = self.zone(n, 'Near').msg.length
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        self.rules(n)
        self.run_worker(n)
        self.assertEqual(self.zone(n, 'Near').rules_fp, '')
        self.assertEqual(self.zone(n, 'Near').msg.length, length)
        self.assertEqual(self.stored(n).length, length)

    def test_rules_of_another_site_shape_nothing(self):
        n = self.site()
        calls = self.count_regen(n)
        self.rules(n, rule('nogo', 4, 4, 6, 6), site='B')
        self.assertFalse(n._site_map_rules_dirty)
        self.run_worker(n)
        self.assertEqual(calls, [])

    def test_bad_payload_keeps_the_rules(self):
        n = self.site()
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        n.callback_map_rules(T.String('not json'))
        self.assertEqual(len(n._map_rules[1]), 1)

    def test_pickled_zone_of_older_rules_counts_as_behind(self):
        n = self.site()
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        del self.zone(n, 'Far').__dict__['rules_fp']      # a pickle from before
        self.assertEqual(n._zones_behind_rules(), [])
        self.zone(n, 'Near').rules_fp = ''
        self.assertEqual([z.msg.name for z in n._zones_behind_rules()], ['Near'])
        n._site_map_rules_dirty = False
        n.callback_load_planner_data(T.Bool(True))
        self.assertTrue(n._site_map_rules_dirty)

    def test_worker_runs_rules_without_a_grid(self):
        n = self.site()
        raster = n.map_data
        self.rules(n, rule('noblade', 4, 4, 6, 6))
        self.run_worker(n)
        self.assertIs(n.map_data, raster)    # no rebuild for a rule


class RunningMissionTest(NodeCase):

    def mission(self, n, status='Mowing'):
        n.callback_mission_program(T.String('MowAll'))
        n.callback_mission_status(T.String(status))

    def test_rules_wait_for_the_end_of_the_mission(self):
        n = self.site()
        length = self.stored(n).length
        paths = len(self.stored(n).paths)
        self.mission(n)
        calls = self.count_regen(n)
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        self.assertEqual(calls, [])
        self.assertEqual(self.stored(n).length, length)
        self.assertEqual(len(self.stored(n).paths), paths)
        self.assertEqual(len(self.zone(n, 'Near').msg.paths), paths)
        started = []
        n._site_map_timer_fire = lambda e: started.append(True) or n._site_map_worker()
        n.callback_mission_status(T.String('Mowing'))
        self.assertEqual(started, [])
        n.callback_mission_status(T.String('Stopped'))
        self.assertEqual(started, [True])
        self.assertEqual(calls, ['Near'])
        self.assertLess(self.stored(n).length, length)
        n.callback_mission_status(T.String('Ready'))
        self.assertEqual(started, [True])

    def test_edited_map_waits_too_and_the_newest_grid_wins(self):
        n = self.site()
        self.mission(n)
        calls = self.count_regen(n)
        first, newest = T.grid(400), T.grid(400)
        newest.data = list(newest.data)
        newest.data[0] = 100
        n._site_map_pending = first
        self.run_worker(n)
        self.assertEqual(calls, [])
        self.assertIs(n._site_map_pending, first)
        n._site_map_pending = newest
        n._site_map_timer_fire = lambda e: n._site_map_worker()
        n.callback_mission_status(T.String('Stopped'))
        self.assertEqual(sorted(calls), ['Far', 'Near'])
        self.assertIsNone(n._site_map_pending)
        self.assertIs(n.map_data.initial_map, newest)

    def test_busy_mission_without_a_program_also_waits(self):
        n = self.site()
        n.callback_mission_busy(T.Bool(True))
        calls = self.count_regen(n)
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        self.assertEqual(calls, [])
        n._site_map_timer_fire = lambda e: n._site_map_worker()
        n.callback_mission_busy(T.Bool(False))
        self.assertEqual(calls, ['Near'])


class StartTest(NodeCase):
    """What reaches the mission, through the real select callbacks."""

    def setUp(self):
        NodeCase.setUp(self)
        self.addCleanup(setattr, T.M.program_map, 'needs_verify',
                        T.M.program_map.needs_verify)
        T.M.program_map.needs_verify = lambda *a: False     # no navi to ask

    def start(self, n, name='MowAll', resume=False):
        del T.EVENTS[:]
        (n.callback_program_select_resume if resume else n.callback_program_select)(
            T.String(name))
        topic = '/web_plan/program_active' + ('_unfinished' if resume else '')
        return T.last(topic)

    def state(self):
        return json.loads(T.last('/web_plan/program_start_state').data)

    def stop_at(self, n, checkpoint, program='MowAll'):
        """The mission reports where it stopped."""
        p = [p for p in n.permanent_program_list_msg.program_list if p.name == program][0]
        p.last_result = checkpoint

    def names(self, msg):
        return [z.name for z in msg.zone_list]

    # ---- a zone a rule took away
    def test_zone_under_a_rule_is_left_out_with_the_reason(self):
        n = self.site()
        self.rules(n, rule('nogo', 1, 1, 9, 9, 'Pond'))
        self.run_worker(n)
        msg = self.start(n)
        self.assertEqual(self.names(msg), ['Far'])
        self.assertEqual(msg.length, self.zone(n, 'Far').msg.length)
        note = self.state()['notes'][0]
        self.assertEqual((note['code'], note['zone'], note['program']),
                         ('zone_under_rule', 'Near', 'MowAll'))
        self.assertIn('Pond', note['text_en'])
        self.assertIn('Pond', note['text_cs'])
        self.assertEqual(self.state()['state'], T.M.program_map.READY)
        self.assertEqual(T.mem_programs(n)['MowAll'], ['Near', 'Far'])    # it stays

    def test_zone_is_back_when_the_temporary_rule_ends(self):
        n = self.site()
        self.rules(n, rule('nogo', 1, 1, 9, 9), seq=1)
        self.run_worker(n)
        self.rules(n, seq=2)
        self.run_worker(n)
        self.assertEqual(self.names(self.start(n)), ['Near', 'Far'])
        self.assertNotIn('notes', self.state())

    def test_nothing_left_refuses_with_the_reason(self):
        n = self.site()
        self.rules(n, rule('nogo', 1, 1, 9, 9, 'Pond'), rule('nogo', 11, 11, 17, 17, 'Bed'))
        self.run_worker(n)
        self.assertIsNone(self.start(n))
        state = self.state()
        self.assertEqual(state['code'], 'program_unusable')
        self.assertIn('Pond', state['text_en'])
        self.assertIn('Bed', state['text_en'])

    def test_no_path_without_a_rule_still_refuses(self):
        n = self.site()
        self.zone(n, 'Near').msg.paths = []
        self.assertIsNone(self.start(n))
        self.assertIn("zone 'Near' has no mowing paths", self.state()['text_en'])

    def test_chain_reports_a_left_out_zone(self):
        n = self.site()
        n.callback_map_save_zone(T.zmsg('Other', 2, 12, 8, 18))
        n.callback_program_new(T.program('Next', T.zone_msgs(n, 'Near', 'Other')))
        self.rules(n, rule('nogo', 1, 1, 9, 9, 'Pond'))
        self.run_worker(n)
        n.callback_program_select(T.String('MowAll>>>Next'))
        chain = json.loads(T.last('/web_plan/program_chain_state').data)
        self.assertEqual(chain['programs'], ['MowAll', 'Next'])
        self.assertEqual([(x['program'], x['zone']) for x in chain['notes']],
                         [('MowAll', 'Near'), ('Next', 'Near')])
        listed = T.last('/web_plan/program_chain').program_list
        self.assertEqual([self.names(p) for p in listed], [['Far'], ['Other']])

    # ---- a checkpoint on other paths
    def test_checkpoint_holds_on_the_same_paths(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        n.callback_map_rules(T.String(rules_json([rule('slow', 0, 0, 1, 1)])))
        msg = self.start(n, resume=True)
        self.assertEqual(msg.last_result, 'failed: on_path-Far-3-40')
        self.assertNotIn('notes', self.state())

    def test_regenerated_identically_keeps_the_checkpoint(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        n._do_site_map_refresh_locked(T.grid(400), 0)
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Far-3-40')

    def test_changed_paths_restart_that_zone(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        self.rules(n, rule('noblade', 13, 13, 14, 14))
        self.run_worker(n)
        msg = self.start(n, resume=True)
        self.assertEqual(msg.last_result, 'failed: on_path-Far-0-0')
        note = self.state()['notes'][0]
        self.assertEqual((note['code'], note['zone']), ('checkpoint_stale', 'Far'))
        saved = n.permanent_program_list_msg.program_list[0].last_result
        self.assertEqual(saved, 'failed: on_path-Far-0-0')
        self.assertEqual(T.M.vbundle.load_programs('A')[0]['last_result'], saved)

    def test_late_write_back_of_the_old_checkpoint_changes_nothing(self):
        n = self.site()
        self.start(n)
        self.rules(n, rule('noblade', 13, 13, 14, 14))
        self.run_worker(n)
        late = T.program('MowAll')
        late.last_result = 'failed: on_path-Far-3-40'
        n._mission_write_back(late)     # the regen came first, the report after
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Far-0-0')

    def test_changed_zone_ahead_of_the_checkpoint_is_not_skipped(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        self.rules(n, rule('noblade', 4, 4, 5, 5))
        self.run_worker(n)
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Near-0-0')

    def test_changed_zone_behind_the_checkpoint_is_no_reason(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Near-3-40')
        self.rules(n, rule('noblade', 13, 13, 14, 14))
        self.run_worker(n)
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Near-3-40')

    def test_no_record_counts_as_changed(self):
        n = self.site()
        self.stop_at(n, 'failed: on_path-Near-3-40')       # stopped before this version
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Near-0-0')

    def test_record_survives_a_restart(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        n._run_paths = n._run_paths_read()
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Far-3-40')

    def test_rebased_checkpoint_stays_rebased_after_the_next_stop(self):
        n = self.site()
        self.start(n)
        self.rules(n, rule('noblade', 13, 13, 14, 14))
        self.run_worker(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        self.start(n, resume=True)
        self.stop_at(n, 'failed: on_path-Far-2-10')     # that run stopped again
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Far-2-10')

    def test_checkpoint_in_a_left_out_zone_goes_on_with_the_next(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Near-3-40')
        self.rules(n, rule('nogo', 1, 1, 9, 9))
        self.run_worker(n)
        msg = self.start(n, resume=True)
        self.assertEqual(self.names(msg), ['Far'])
        self.assertEqual(msg.last_result, 'failed: on_path-Far-0-0')

    def test_checkpoint_in_the_last_zone_left_out_mows_what_remains(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Far-3-40')
        self.rules(n, rule('nogo', 11, 11, 17, 17))
        self.run_worker(n)
        msg = self.start(n, resume=True)
        self.assertEqual(self.names(msg), ['Near'])
        codes = [x['code'] for x in self.state()['notes']]
        self.assertIn('checkpoint_zone_gone', codes)

    def test_zone_left_out_of_the_stopped_run_is_not_skipped_later(self):
        n = self.site()
        self.rules(n, rule('nogo', 1, 1, 9, 9), seq=1)
        self.run_worker(n)
        self.start(n)                                   # Near left out
        self.stop_at(n, 'failed: on_path-Far-3-40')
        self.rules(n, seq=2)                            # the rule ended
        self.run_worker(n)
        self.assertEqual(self.start(n, resume=True).last_result, 'failed: on_path-Near-0-0')

    def test_zone_op_on_the_checkpoint_zone_mows_the_whole_program(self):
        n = self.site()
        self.start(n)
        self.stop_at(n, 'failed: on_path-Near-3-40')
        n.callback_zone_op(T.String(json.dumps(
            {'req': 1, 'op': 'split', 'name': 'Near', 'line': [[4, 0], [4, 10]],
             'new_names': ['East', 'West']})))
        msg = self.start(n, resume=True)
        self.assertEqual(self.names(msg), ['East', 'West', 'Far'])
        self.assertEqual(self.state()['notes'][0]['code'], 'checkpoint_zone_gone')

    def test_a_plain_run_is_not_touched(self):
        n = self.site()
        self.stop_at(n, 'failed: on_path-Near-3-40')
        self.assertEqual(self.start(n).last_result, 'failed: on_path-Near-3-40')


class ZoneCheckTest(NodeCase):

    def req(self, x0=2, y0=2, x1=8, y1=8, **kw):
        out = {'req': 'r1', 'points': [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
               'settings': {'border_paths': 1, 'paths_distance': 0.3, 'simplify': 0.05}}
        out.update(kw)
        return out

    def check(self, n, **kw):
        return n._zone_check(self.req(**kw), lambda: False)

    def codes(self, out):
        return [p['code'] for p in out['problems']]

    def wait_results(self, count):
        end = time.time() + 20
        while time.time() < end:
            got = [json.loads(m.data) for t, m in T.EVENTS
                   if t == '/web_plan/zone_check_result']
            if len(got) >= count:
                return got
            time.sleep(0.01)
        self.fail('no result')

    def test_figures_match_a_save(self):
        n = self.site()
        out = self.check(n)
        z = self.zone(n, 'Near').msg
        self.assertTrue(out['ok'], out['problems'])
        self.assertEqual(out['req'], 'r1')
        self.assertEqual(out['length_m'], z.length)
        self.assertAlmostEqual(out['mowable_m2'], z.area, delta=2.5)
        self.assertAlmostEqual(out['mowable_m2'] + out['unreachable_m2'], 36, delta=0.2)
        self.assertGreater(len(out['preview']), 1)
        self.assertEqual(len(out['preview'][0][0]), 2)
        self.assertGreater(out['minutes'], 0)
        self.assertEqual(out['minutes_source'], 'speed')
        json.dumps(out)

    def test_saves_nothing(self):
        n = self.site()
        n.callback_zone_selected(T.String('Far'))
        selected = n.current_zone
        zones = list(n.map_data.zones)
        data = n.map_data
        zones_file = os.path.join(T.site_dir('A'), 'zones.geojson')
        with open(zones_file) as f:
            before = f.read()
        del T.EVENTS[:]
        self.check(n, x0=1, name='New')
        self.assertIs(n.current_zone, selected)
        self.assertIs(n.map_data, data)
        self.assertEqual(n.map_data.zones, zones)
        with open(zones_file) as f:
            self.assertEqual(f.read(), before)
        self.assertEqual([t for t, m in T.EVENTS], [])

    def test_rule_lowers_mowable_and_is_reported(self):
        n = self.site()
        plain = self.check(n)
        self.rules(n, rule('nogo', 4, 4, 6, 6, 'Pond'))
        out = self.check(n)
        self.assertLess(out['mowable_m2'], plain['mowable_m2'] - 4)
        self.assertGreater(out['unreachable_m2'], plain['unreachable_m2'] + 4)
        self.assertLess(out['length_m'], plain['length_m'])
        over = [p for p in out['problems'] if p['code'] == 'overlaps_nogo'][0]
        self.assertEqual(over['severity'], 'warning')
        self.assertEqual(over['params']['name'], 'Pond')
        self.assertAlmostEqual(over['params']['area_m2'], 4)
        self.assertTrue(out['ok'])

    def test_nogo_wall_cuts_a_part_off_noblade_does_not(self):
        n = self.site()
        self.rules(n, rule('noblade', 4.5, -5, 5.5, 25))
        out = self.check(n)
        self.assertIn('zone_split', self.codes(out))
        self.assertNotIn('part_unreachable', self.codes(out))
        self.rules(n, rule('nogo', 4.5, -5, 5.5, 25), seq=2)
        out = self.check(n)
        self.assertIn('part_unreachable', self.codes(out))
        self.assertTrue(out['ok'])

    def test_refusals_are_errors_with_codes(self):
        n = self.site()
        bad = self.req()
        bad['settings']['cut_height'] = 90
        out = n._zone_check(bad, lambda: False)
        self.assertFalse(out['ok'])
        p = out['problems'][0]
        self.assertEqual((p['code'], p['params']['max']), ('cut_height_range', 80))
        out = n._zone_check({'req': 'x', 'points': [[0, 0], [1, 1]]}, lambda: False)
        self.assertEqual(self.codes(out), ['bad_points'])
        bow = {'req': 'x', 'points': [[1, 1], [5, 5], [5, 1], [1, 5]]}
        self.assertEqual(self.codes(n._zone_check(bow, lambda: False)), ['self_intersecting'])
        out = n._zone_check({'points': [[1, 1], [2, 1], [2, 2]],
                             'settings': {'border_paths': 'x'}}, lambda: False)
        self.assertEqual(self.codes(out), ['bad_settings'])

    def test_no_path_outside_the_free_space(self):
        n = self.site()
        out = self.check(n, x0=300, y0=300, x1=304, y1=304)
        self.assertFalse(out['ok'])
        self.assertIn('no_path', self.codes(out))
        self.assertIn('outside_map', self.codes(out))

    def test_no_map(self):
        n = T.new_node()
        self.assertEqual(self.codes(self.check(n)), ['no_map'])

    def test_running_program_and_unloaded_programs(self):
        n = self.site()
        n.callback_mission_program(T.String('MowAll'))
        n.callback_mission_status(T.String('Mowing'))
        out = self.check(n, name='Near')
        self.assertFalse(out['ok'])
        self.assertEqual(out['problems'][0]['params']['program'], 'MowAll')
        self.assertTrue(self.check(n, name='Free').get('ok'))
        n._programs_need_import = True
        self.assertIn('programs_not_loaded', self.codes(self.check(n)))

    def test_overlap_with_another_zone_but_not_itself(self):
        n = self.site()
        self.assertNotIn('overlaps_zone', self.codes(self.check(n, name='Near')))
        self.assertEqual(
            [p['params']['name'] for p in self.check(n)['problems']
             if p['code'] == 'overlaps_zone'], ['Near'])

    def test_minutes_from_history(self):
        n = self.site()
        p = n.permanent_program_list_msg.program_list[0]
        p.last_duration_minutes, p.last_result, p.speed = 30, 'succeeded', 'slow'
        out = self.check(n, name='Near')
        self.assertEqual(out['minutes_source'], 'history')
        expected = round(30.0 / p.length * out['length_m'])
        self.assertEqual(out['minutes'], expected)

    def test_callback_returns_at_once_and_answers_each_request(self):
        n = self.site()
        release = threading.Event()
        real = n._zone_check

        def slow(req, cancelled):
            release.wait(10)
            return real(req, cancelled)
        n._zone_check = slow
        del T.EVENTS[:]
        t0 = time.time()
        for i in range(4):
            n.callback_zone_check(T.String(json.dumps(self.req(req=i))))
        self.assertLess(time.time() - t0, 0.5)
        release.set()
        got = self.wait_results(4)
        by_req = {r['req']: r for r in got}
        self.assertEqual(sorted(by_req), [0, 1, 2, 3])
        self.assertTrue(by_req[3]['ok'])
        self.assertNotIn('cancelled', by_req[3])
        self.assertTrue(all(by_req[i]['cancelled'] for i in (0, 1, 2)))

    def test_superseded_run_stops_inside_generation(self):
        n = self.site()
        calls = []

        def cancelled():
            calls.append(1)
            return len(calls) > 2
        with self.assertRaises(cr.Cancelled):
            n._zone_check(self.req(), cancelled)
        self.assertEqual(len(calls), 3)

    def test_bad_json_gets_a_result(self):
        n = self.site()
        del T.EVENTS[:]
        n.callback_zone_check(T.String('{'))
        self.assertEqual(self.wait_results(1)[0]['problems'][0]['code'], 'bad_request')

    def test_broken_check_answers_and_the_next_one_runs(self):
        n = self.site()
        real = n._zone_check
        n._zone_check = lambda req, c: 1 / 0
        del T.EVENTS[:]
        n.callback_zone_check(T.String(json.dumps(self.req())))
        self.assertEqual(self.wait_results(1)[0]['problems'][0]['code'], 'internal')
        n._zone_check = real
        for _ in range(500):
            if not n._zone_check_active:
                break
            time.sleep(0.01)
        n.callback_zone_check(T.String(json.dumps(self.req())))
        self.assertTrue(self.wait_results(2)[1]['ok'])


class ZoneOpTest(NodeCase):

    def op(self, n, **req):
        del T.EVENTS[:]
        req.setdefault('req', 'r')
        n.callback_zone_op(T.String(json.dumps(req)))
        results = [json.loads(m.data) for t, m in T.EVENTS if t == '/web_plan/zone_op_result']
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]['req'], req['req'])
        return results[0]

    def bundle_names(self):
        return sorted(z['name'] for z in T.M.vbundle.load_zones('A'))

    def test_split_updates_programs_and_bundle(self):
        n = self.site()
        r = self.op(n, op='split', name='Near', line=[[4, 0], [4, 10]])
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['names'], ['Near', 'Near 2'])
        self.assertEqual(r['removed'], [])
        self.assertEqual(r['programs'], 1)
        self.assertEqual(T.mem_programs(n)['MowAll'], ['Near', 'Near 2', 'Far'])
        self.assertEqual(T.yaml_programs('A'), {'MowAll': ['Near', 'Near 2', 'Far']})
        self.assertEqual(self.bundle_names(), ['Far', 'Near', 'Near 2'])
        self.assertTrue(self.stored(n, 'Near 2').paths)
        self.assertEqual(T.listed_zones(), ['Far', 'Near', 'Near 2'])
        p = n.permanent_program_list_msg.program_list[0]
        self.assertEqual(p.length, sum(z.length for z in p.zone_list))

    def test_split_keeps_settings(self):
        n = self.site()
        z = T.zmsg('Tall', 2, 12, 8, 18)
        z.cut_height, z.rpm = 45, 3100
        n.callback_map_save_zone(z)
        self.assertTrue(self.op(n, op='split', name='Tall', line=[[5, 11], [5, 19]])['ok'])
        self.assertEqual((self.zone(n, 'Tall 2').msg.cut_height,
                          self.zone(n, 'Tall 2').msg.rpm), (45, 3100))

    def test_split_with_given_names_removes_the_old_one(self):
        n = self.site()
        r = self.op(n, op='split', name='Near', line=[[4, 0], [4, 10]],
                    new_names=['East', 'West'])
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['removed'], ['Near'])
        self.assertEqual(T.yaml_programs('A'), {'MowAll': ['East', 'West', 'Far']})
        self.assertEqual(self.bundle_names(), ['East', 'Far', 'West'])

    def test_merge_updates_programs_and_bundle(self):
        n = self.site()
        n.callback_map_save_zone(T.zmsg('Side', 8, 2, 11, 8))
        r = self.op(n, op='merge', names=['Near', 'Side'])
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['names'], ['Near'])
        self.assertEqual(r['removed'], ['Side'])
        self.assertEqual(self.bundle_names(), ['Far', 'Near'])
        area = geometry.Polygon([(p.x, p.y) for p in
                                 self.zone(n, 'Near').msg.polygon.polygon.points]).area
        self.assertAlmostEqual(area, 54, places=1)

    def test_merge_of_two_zones_of_one_program_leaves_one_entry(self):
        n = self.site()
        n.callback_map_save_zone(T.zmsg('Side', 8, 2, 11, 8))
        n.callback_program_new(T.program('Two', T.zone_msgs(n, 'Side', 'Far', 'Near')))
        r = self.op(n, op='merge', names=['Near', 'Side'], new_name='Big')
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['programs'], 2)
        self.assertEqual(T.mem_programs(n)['Two'], ['Big', 'Far'])
        self.assertEqual(T.mem_programs(n)['MowAll'], ['Big', 'Far'])
        self.assertEqual(sorted(r['removed']), ['Near', 'Side'])

    def test_duplicate_leaves_programs_alone(self):
        n = self.site()
        r = self.op(n, op='duplicate', name='Near')
        self.assertTrue(r['ok'], r)
        self.assertEqual(r['names'], ['Near copy'])
        self.assertEqual(r['programs'], 0)
        self.assertEqual(self.bundle_names(), ['Far', 'Near', 'Near copy'])
        self.assertEqual(T.mem_programs(n)['MowAll'], ['Near', 'Far'])
        self.assertTrue(self.zone(n, 'Near copy').msg.paths)
        self.assertEqual(self.op(n, op='duplicate', name='Near')['names'], ['Near copy 2'])

    def test_generated_zones_keep_the_rules(self):
        n = self.site()
        self.rules(n, rule('nogo', 4, 4, 6, 6))
        self.run_worker(n)
        self.assertTrue(self.op(n, op='duplicate', name='Near')['ok'])
        self.assertNotEqual(self.zone(n, 'Near copy').rules_fp, '')

    def test_refused_while_a_program_runs_on_it(self):
        n = self.site()
        n.callback_mission_program(T.String('MowAll'))
        n.callback_mission_status(T.String('Mowing'))
        r = self.op(n, op='split', name='Near', line=[[4, 0], [4, 10]])
        self.assertEqual((r['ok'], r['code']), (False, 'program_running'))
        self.assertIn('MowAll', r['reason'])
        n.callback_map_save_zone(T.zmsg('Side', 18, 2, 19.8, 8))
        self.assertEqual(self.op(n, op='merge', names=['Side', 'Near'])['code'],
                         'program_running')
        self.assertEqual(T.mem_programs(n)['MowAll'], ['Near', 'Far'])
        self.assertTrue(self.op(n, op='duplicate', name='Side')['ok'])

    def test_refused_while_programs_are_not_loaded(self):
        n = self.site()
        n._programs_need_import = True
        r = self.op(n, op='duplicate', name='Near')
        self.assertEqual(r['code'], 'programs_not_loaded')

    def test_refusals_change_nothing(self):
        n = self.site()
        zones = list(n.map_data.zones)
        cases = [
            (dict(op='split', name='Nope', line=[[0, 0], [1, 1]]), 'not_found'),
            (dict(op='split', name='Near', line=[[30, 0], [30, 5]]), 'line_misses_zone'),
            (dict(op='split', name='Near'), 'bad_line'),
            (dict(op='merge', names=['Near', 'Far']), 'not_adjacent'),
            (dict(op='merge', names=['Near']), 'bad_request'),
            (dict(op='merge', names=['Near', 'Near']), 'bad_request'),
            (dict(op='duplicate', name='Near', new_name='far'), 'name'),
            (dict(op='turn', name='Near'), 'bad_request'),
            (dict(op='duplicate'), 'bad_request'),
        ]
        for req, code in cases:
            r = self.op(n, **req)
            self.assertEqual((r['ok'], r['code']), (False, code), req)
            self.assertTrue(r['reason'])
        self.assertEqual(n.map_data.zones, zones)
        self.assertEqual(self.bundle_names(), ['Far', 'Near'])

    def test_part_without_a_path_refuses_all_of_it(self):
        n = self.site()
        zones = list(n.map_data.zones)
        r = self.op(n, op='split', name='Near', line=[[2.3, 0], [2.3, 10]])
        self.assertEqual((r['ok'], r['code']), (False, 'no_path'))
        self.assertEqual(n.map_data.zones, zones)

    def test_no_map(self):
        n = T.new_node()
        self.assertEqual(self.op(n, op='duplicate', name='Near')['code'], 'no_map')

    def test_bad_json(self):
        n = self.site()
        del T.EVENTS[:]
        n.callback_zone_op(T.String('['))
        got = [json.loads(m.data) for t, m in T.EVENTS if t == '/web_plan/zone_op_result']
        self.assertEqual(got[0]['code'], 'bad_request')

    def test_internal_error_gives_one_result(self):
        n = self.site()
        n._zone_op_commit = lambda *a: 1 / 0
        self.assertEqual(self.op(n, op='duplicate', name='Near')['code'], 'failed')


class HeightAndEffectiveTest(NodeCase):

    def result(self):
        return json.loads(T.last('/web_plan/zone_result').data)

    def props(self, name):
        return [z['props'] for z in T.M.vbundle.load_zones('A') if z['name'] == name][0]

    def test_zone_save_range(self):
        n = self.site()
        for height, ok in ((19, False), (81, False), (20, True), (80, True), (0, True)):
            z = T.zmsg('H', 2, 12, 8, 18)
            z.cut_height = height
            n.callback_map_save_zone(z)
            self.assertEqual(self.result()['ok'], ok, height)
            if not ok:
                self.assertIn('20-80 mm', self.result()['message'])

    def test_program_range_only_counts_when_it_overrides(self):
        n = self.site()
        p = T.program('High', T.zone_msgs(n, 'Near'))
        p.cut_height, p.override_zone = 90, False
        n.callback_program_new(p)
        self.assertIn('High', T.mem_programs(n))
        p = T.program('Higher', T.zone_msgs(n, 'Near'))
        p.cut_height, p.override_zone = 90, True
        n.callback_program_new(p)
        self.assertNotIn('Higher', T.mem_programs(n))

    def test_stored_height_out_of_range_does_not_start(self):
        n = self.site()
        program = n.permanent_program_list_msg.program_list[0]
        self.zone(n, 'Near').msg.cut_height = 90
        self.assertIn('outside 20-80 mm', n._program_with_current_zones(program)[1])
        program.override_zone, program.cut_height = True, 60
        self.assertIsNotNone(n._program_with_current_zones(program)[0])

    def test_mission_write_back_is_not_validated(self):
        n = self.site()
        p = n.permanent_program_list_msg.program_list[0]
        p.cut_height, p.override_zone = 90, True
        reported = T.program('MowAll')
        reported.last_result, reported.cut_height, reported.override_zone = 'succeeded', 90, True
        n._mission_write_back(reported)
        self.assertEqual(p.last_result, 'succeeded')

    def test_effective_follows_the_program(self):
        n = self.site()
        z = T.zmsg('Near', 2, 2, 8, 8)
        z.cut_height, z.rpm = 45, 3000
        n.callback_map_save_zone(z)
        self.assertNotIn('effective', self.props('Near'))
        p = T.program('Over', T.zone_msgs(n, 'Near'))
        p.cut_height, p.rpm, p.override_zone = 60, 0, True
        n.callback_program_new(p)
        self.assertEqual(self.props('Near')['effective'],
                         [{'program': 'Over', 'cut_height': 60, 'rpm': 3000}])
        self.assertNotIn('effective', self.props('Far'))
        n.callback_map_save_zone(z)                     # a zone export keeps it
        self.assertEqual(len(self.props('Near')['effective']), 1)
        n.callback_program_remove(T.String('Over'))
        self.assertNotIn('effective', self.props('Near'))

    def test_same_values_are_no_override(self):
        rows = [{'name': 'Z', 'props': {'cut_height': 45, 'rpm': 3000}}]
        progs = [{'name': 'P', 'zone_names': ['Z'], 'override_zone': True,
                  'cut_height': 45, 'rpm': 0},
                 {'name': 'Q', 'zone_names': ['Z'], 'override_zone': False,
                  'cut_height': 60, 'rpm': 0}]
        self.assertFalse(T.M.bundle_sync.apply_effective(rows, progs))
        self.assertNotIn('effective', rows[0]['props'])

    def test_effective_is_not_a_change_of_the_import(self):
        n = self.site()
        p = T.program('Over', T.zone_msgs(n, 'Near'))
        p.cut_height, p.override_zone = 60, True
        n.callback_program_new(p)
        stamp = n._bundle_stamp_get('A', 'zones')
        self.assertIsNotNone(stamp)
        self.assertEqual(stamp, T.M.vbundle.file_mtime('A', 'zones'))

    def test_republish_sends_map_data(self):
        n = self.site()
        del T.EVENTS[:]
        n.callback_republish(T.Bool(True))
        self.assertIsNotNone(T.last('/web_plan/map_data'))


if __name__ == '__main__':
    unittest.main()
