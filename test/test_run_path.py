#!/usr/bin/env python3
"""run_path: start / end of a run, what the markers hold, Show against the run.
No ROS master needed.  Run:  python3 test/test_run_path.py"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

import rospy
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from vitulus_msgs.msg import MapEditZone, PlannerProgram

from node_planner import run_path

STAMP = rospy.Time(0)


def path(*xy):
    msg = Path()
    for x, y in xy:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        msg.poses.append(pose)
    return msg


def zone(name, *paths):
    msg = MapEditZone()
    msg.name = name
    msg.polygon.polygon.points = [Point32(0, 0, 0), Point32(9, 0, 0), Point32(9, 9, 0)]
    msg.paths = list(paths)
    return msg


def program(*zones):
    msg = PlannerProgram()
    msg.name = 'P'
    msg.zone_list = list(zones)
    return msg


RING = ((0, 0), (5, 0), (5, 5), (0, 0))
PASS = ((1, 1), (4, 1), (4, 2), (1, 2))


class RunningProgram(unittest.TestCase):
    def test_table(self):
        for busy, name, status, want in (
                (True, 'P', 'Mowing started', 'P'),
                (True, 'P', 'Returning to dock', 'P'),   # battery, weather: on the way
                (True, ' ', 'Returning to dock', ''),    # finished, on the way home
                (True, 'P', 'Stopped', ''),              # busy lags 0.5 s after STOP
                (False, 'P', 'Stopped', ''),
                (False, 'P', 'TERMINAL ERROR', ''),
                (False, 'P', 'Initiating', 'P'),         # before busy
                (False, 'P', 'Initiating unfinished', 'P'),
                (None, 'P', 'Initiating', 'P'),
                (False, 'P', 'Ready', ''),               # dock reached, the name stayed
                (False, 'P', 'Undocking', 'P'),          # busy still on its way
                (False, ' ', 'Ready', ''),
                (False, ' ', 'Rejected: no GPS', ''),
                (None, '', '', '')):
            self.assertEqual(run_path.running_program(busy, name, status), want, (busy, name, status))


class Checkpoint(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(run_path.parse_checkpoint('failed: on_path-Zone-A-3-12'), ('Zone-A', 3, 12))
        self.assertEqual(run_path.parse_checkpoint('failed: on_path-Z-3'), ('Z', 3, 0))
        for text in ('succeeded', 'failed: on_init', '', None, 'failed: on_path-Z'):
            self.assertIsNone(run_path.parse_checkpoint(text))


class Markers(unittest.TestCase):
    def layers(self, prg, checkpoint=None):
        markers = run_path.build_markers(prg, STAMP, checkpoint)[0]
        return {m.ns: m for m in markers}

    def test_layers(self):
        got = self.layers(program(zone('A', path(*RING), path(*PASS))))
        self.assertEqual(sorted(got), ['run_0_border', 'run_0_fill', 'run_0_outline'])
        self.assertEqual(len(got['run_0_outline'].points), 6)
        self.assertEqual(len(got['run_0_fill'].points), 6)
        self.assertTrue(all(m.type == Marker.LINE_LIST and m.header.frame_id == 'map'
                            for m in got.values()))
        self.assertEqual(len({(m.ns, m.id) for m in got.values()}), 3)

    def test_checkpoint_splits_inside_the_path(self):
        prg = program(zone('A', path(*RING), path(*PASS)), zone('B', path(*PASS)))
        got = self.layers(prg, ('A', 1, 2))
        # the whole ring + PASS[0..2]; the rest of PASS from the pose it stands at
        self.assertEqual(len(got['run_0_done'].points), 6 + 4)
        self.assertEqual(len(got['run_0_fill'].points), 2)
        self.assertNotIn('run_0_outline', got)
        self.assertEqual(len(got['run_1_fill'].points), 6)
        self.assertNotIn('run_1_done', got)
        done, total = run_path.build_markers(prg, STAMP, ('A', 1, 2))[1:]
        self.assertAlmostEqual(total, run_path.build_markers(prg, STAMP)[2])
        self.assertLess(done, total)

    def test_checkpoint_of_another_path_set_marks_nothing(self):
        prg = program(zone('A', path(*PASS)))
        self.assertEqual(self.layers(prg, ('A', 7, 0)).keys(), self.layers(prg).keys())
        self.assertEqual(self.layers(prg, ('Gone', 0, 0)).keys(), self.layers(prg).keys())

    def test_two_programs_do_not_share_a_marker_key(self):
        prg = program(zone('A', path(*PASS)))
        run = run_path.build_markers(prg, STAMP)[0]
        manual = run_path.build_markers(prg, STAMP, prefix='manual')[0]
        self.assertFalse({(m.ns, m.id) for m in run} & {(m.ns, m.id) for m in manual})

    def test_empty_things_do_not_raise(self):
        self.assertEqual(self.layers(PlannerProgram()), {})
        bare = zone('A', Path(), path((1, 1)))
        bare.polygon.polygon.points = []
        self.assertEqual(self.layers(program(bare)), {})

    def test_big_program_is_thinned_and_keeps_the_ends(self):
        line = [(i * 0.1, 0.0) for i in range(120001)]
        got = self.layers(program(zone('A', path(*line))))['run_0_fill']
        self.assertLessEqual(len(got.points) / 2, run_path.MAX_SEGMENTS)
        self.assertAlmostEqual(got.points[-1].x, line[-1][0])


class Shown(unittest.TestCase):
    def setUp(self):
        self.s = run_path.ShownPaths()

    def state(self):
        return json.loads(self.s.state_json())

    def test_hide_holds_for_this_run_only(self):
        self.assertTrue(self.s.set_run('P'))
        self.assertFalse(self.s.set_run('P'))
        self.s.set_run_visible(False)
        self.assertFalse(self.state()['run_visible'])
        self.assertTrue(self.s.set_run(''))
        self.s.set_run('P')
        self.assertTrue(self.state()['run_visible'])

    def test_show_of_the_running_program_brings_it_back(self):
        self.s.set_run('P')
        self.s.set_run_visible(False)
        self.s.show_manual('P', [1])
        self.assertTrue(self.state()['run_visible'])
        self.assertEqual(self.state()['manual'], '')

    def test_other_program_waits_for_the_end_of_the_run(self):
        self.s.show_manual('Q', [1])
        self.s.set_run('P')
        self.assertEqual((self.s.manual_visible(), self.state()['manual']), ([], ''))
        self.s.set_run('')
        self.assertEqual((self.s.manual_visible(), self.state()['manual']), ([1], 'Q'))

    def test_same_program_ends_with_the_run(self):
        self.s.show_manual('P', [1])
        self.s.set_run('P')
        self.s.set_run('')
        self.assertEqual((self.s.manual_visible(), self.state()['manual']), ([], ''))

    def test_show_during_a_run_goes_on_top_and_stays_after_it(self):
        self.s.set_run('P')
        self.s.show_manual('agent:Z', [1])
        self.assertEqual(self.s.manual_visible(), [1])
        self.assertTrue(self.state()['run_visible'])
        self.s.set_run('')
        self.assertEqual(self.s.manual_visible(), [1])

    def test_no_run_nothing_to_hide(self):
        self.s.set_run_visible(False)
        self.s.set_run('P')
        self.assertTrue(self.state()['run_visible'])


if __name__ == '__main__':
    unittest.main()
