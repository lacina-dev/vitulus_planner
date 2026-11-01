#!/usr/bin/env python3
"""A program starts on its own map (vitulus-field#46): the decisions and the
wait of node_planner/program_map.py. No ROS. Run: python3 test/test_program_map.py"""
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from node_planner import program_map as pm  # noqa: E402

CAT = {'BACK': ['B1Test', 'Shared'], 'LADA': [], 'T1': ['Shared']}


def plan(ref, served='LADA', listed=None, legacy=False, active='SITE', cat=CAT):
    return pm.plan_start(ref, legacy, served, active, listed or {}, cat)


class Plan(unittest.TestCase):
    def test_other_map_switches(self):
        p = plan('B1Test')
        self.assertEqual((p.kind, p.site, p.name), ('switch', 'BACK', 'B1Test'))

    def test_no_map_on_switches(self):
        self.assertEqual(plan('B1Test', served=None).kind, 'switch')

    def test_active_map_program_is_direct(self):
        p = plan('B1Test', served='BACK', listed={'B1Test': 'BACK'})
        self.assertEqual((p.kind, p.site), ('direct', 'BACK'))

    def test_active_map_wins_a_shared_name(self):
        self.assertEqual(plan('Shared', served='BACK', listed={'Shared': 'BACK'}).kind, 'direct')

    def test_shared_name_elsewhere_is_ambiguous(self):
        p = plan('Shared')
        self.assertEqual((p.kind, p.code), ('refuse', 'ambiguous'))
        self.assertIn('BACK, T1', p.params['sites'])

    def test_ref_picks_the_map(self):
        p = plan('T1|||Shared', served='BACK', listed={'Shared': 'BACK'})
        self.assertEqual((p.kind, p.site, p.name), ('switch', 'T1', 'Shared'))

    def test_ref_to_the_map_that_is_on(self):
        p = plan('BACK|||Shared', served='BACK', listed={'Shared': 'BACK'})
        self.assertEqual(p.kind, 'direct')

    def test_ref_program_not_there(self):
        p = plan('LADA|||Shared', served='BACK')
        self.assertEqual((p.kind, p.code), ('refuse', 'not_on_map'))

    def test_unknown_goes_to_the_planner(self):
        self.assertEqual(plan('Nope').kind, 'direct')

    def test_no_catalog_keeps_the_old_path(self):
        self.assertEqual(plan('B1Test', cat=None).kind, 'direct')

    def test_old_data_on_site_map(self):
        self.assertEqual(plan('Old', served='BACK', listed={'Old': 'SITE'}).kind, 'direct')

    def test_legacy_program_on_site_robot_refused(self):
        p = plan('Old', listed={'Old': 'Old***env*OUTDOOR'})
        self.assertEqual((p.kind, p.code, p.site), ('refuse', 'legacy_program', 'Old'))

    def test_legacy_mode(self):
        name = 'B***env*OUTDOOR'
        p = plan('X', legacy=True, active='A***env*OUTDOOR', listed={'X': name})
        self.assertEqual((p.kind, p.site), ('switch', name))
        self.assertEqual(plan('X', legacy=True, active=name, listed={'X': name}).kind, 'direct')


class Verify(unittest.TestCase):
    def test_table(self):
        good = {'ready': True, 'site': 'A', 'dock_err_m': 0.02}
        for ready, dock, want in (
                (None, 0, False),                                   # navi silent
                (good, 0, False),
                (dict(good, ready=False), 0, True),                 # never taken over
                (dict(good, site='B'), 0, True),                    # old map in navi
                (dict(good, dock_err_m=3.0), 0, True),              # pose not on the dock
                (dict(good, dock_err_m=30.0), 3, False),            # out on the lawn
                (dict(good, ready=False), 3, False),                # out: starts as ever
                (dict(good, site='B'), 3, False),
                (dict(good, dock_err_m=None), 0, False)):           # no docked point
            self.assertEqual(pm.needs_verify(ready, 'A', dock, 0.5), want, (ready, dock))


class Blocker(unittest.TestCase):
    def test_table(self):
        for args, want in (
                ((0, True, 'Ready', False, False), None),
                ((0, True, '', False, False), None),
                ((0, True, 'Rejected: x', False, False), None),
                ((3, False, 'Ready', False, False), 'not_docked'),
                ((0, False, 'Ready', False, False), 'not_docked'),
                ((1, True, 'Ready', False, False), 'dock_busy'),
                ((2, False, 'Ready', False, False), 'dock_busy'),
                ((None, True, 'Ready', False, False), 'dock_unknown'),
                ((0, None, 'Ready', False, False), 'dock_unknown'),
                ((0, True, 'Mowing', True, False), 'mission_running'),
                ((0, True, 'Ready', None, False), 'mission_unknown'),
                ((3, False, 'Stopped', False, False), 'mission_waits_reset'),
                ((0, True, 'TERMINAL ERROR', False, False), 'mission_waits_reset'),
                ((0, True, 'Ready', False, True), 'mapping_running')):
            self.assertEqual(pm.switch_blocker(*args), want, args)


class Env(object):
    """Scripted robot; the clock runs fast."""

    def __init__(self):
        self.t = 1000.0
        self.reports, self.logs, self.requests, self.calls = [], [], [], []
        self.state = (0, True, 'Ready', False, False)
        self.served = 'LADA'
        self.err = None
        self.planner_on = True
        self.navi = {'ready': True, 'site': 'BACK', 'has_dock': True, 'dock_err_m': 0.02}

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s
        time.sleep(0.0005)

    def log(self, text):
        self.logs.append(text)

    def report(self, st):
        self.reports.append(st)

    def robot_state(self):
        return self.state

    def status_seen(self):
        return True

    def request_map(self, site, legacy):
        self.requests.append(site)
        self.served = site
        return 0.0

    def serve_error(self, stamp):
        return self.err

    def map_on(self, site, legacy):
        return self.served == site

    def planner_ready(self, site, legacy):
        return self.planner_on

    def navi_state(self):
        return self.navi


def go(env, timeout=10.0):
    gate = pm.MapGate(env, timeout_s=timeout)
    started = []

    def start(name, verb):
        started.append((name, env.t))
        return True, None
    p = pm.Plan(pm.Plan.SWITCH, 'B1Test', site='BACK')
    gate._run(p, 'B1Test', 'run', False, start)
    return gate, started


class Gate(unittest.TestCase):
    def test_ready(self):
        env = Env()
        _, started = go(env)
        self.assertEqual(env.requests, ['BACK'])
        self.assertEqual([r['state'] for r in env.reports], ['switching', 'ready'])
        self.assertEqual(len(started), 1)

    def test_verify_only_loads_nothing(self):
        env = Env()
        gate = pm.MapGate(env, timeout_s=10.0)
        p = pm.Plan(pm.Plan.SWITCH, 'B1Test', site='BACK')
        p.verify_only = True
        env.served = 'BACK'
        gate._run(p, 'B1Test', 'run', False, lambda n, v: (True, None))
        self.assertEqual(env.requests, [])
        self.assertEqual(env.reports[-1]['state'], 'ready')

    def test_outside_nothing_is_asked(self):
        env = Env()
        env.state = (3, False, 'Ready', False, False)
        _, started = go(env)
        self.assertEqual(env.requests, [])
        self.assertEqual(started, [])
        self.assertEqual(env.reports[-1]['code'], 'not_docked')
        self.assertIn('BACK', env.reports[-1]['text_cs'])
        self.assertIn('v doku', env.reports[-1]['text_cs'])

    def test_serve_error(self):
        env = Env()
        env.err = 'site has no raster'
        _, started = go(env)
        self.assertEqual(started, [])
        last = env.reports[-1]
        self.assertEqual((last['state'], last['code']), ('failed', 'serve_failed'))
        self.assertIn('site has no raster', last['text_cs'])

    def test_timeouts_by_phase(self):
        env = Env()
        env.request_map = lambda site, legacy: 0.0
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'timeout_serve')
        env = Env()
        env.planner_on = False
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'timeout_planner')
        env = Env()
        env.navi = {'ready': False}
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'timeout_navi')

    def test_pose_off_the_dock(self):
        env = Env()
        env.navi['dock_err_m'] = 4.2
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'pose_mismatch')
        self.assertIn('4.2', env.reports[-1]['text_cs'])

    def test_pose_settles(self):
        env = Env()
        env.navi['dock_err_m'] = 4.2
        threading.Timer(0.02, lambda: env.navi.update(dock_err_m=0.03)).start()
        self.assertEqual(len(go(env, timeout=100.0)[1]), 1)

    def test_no_dock_point(self):
        env = Env()
        env.navi['has_dock'] = False
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'no_dock_point')

    def test_no_docked_point(self):
        env = Env()
        env.navi.update(has_docked=False, dock_err_m=None)
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'no_docked_point')

    def test_no_pose_on_the_new_map_is_not_verified(self):
        env = Env()
        env.navi.update(has_docked=True, dock_err_m=None)
        self.assertEqual(go(env)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'timeout_navi')

    def test_settle_starts_again_when_the_planner_drops_out(self):
        env = Env()
        ticks = iter([True, True, False] + [True] * 100000)
        env.planner_ready = lambda site, legacy: next(ticks)
        started = go(env)[1]
        self.assertEqual(len(started), 1)
        # 2 s from the last gap (third poll at 1000.4), not from the first pass
        self.assertGreaterEqual(started[0][1], 1000.0 + 3 * 0.2 + 2.0 - 1e-6)

    def test_robot_leaves_during_the_wait(self):
        env = Env()
        env.navi = {'ready': False}
        threading.Timer(0.02, lambda: setattr(env, 'state', (3, False, 'Ready', False, False))).start()
        self.assertEqual(go(env, timeout=100000.0)[1], [])
        self.assertEqual(env.reports[-1]['code'], 'not_docked')

    def test_cancel(self):
        env = Env()
        env.navi = {'ready': False}
        gate = pm.MapGate(env, timeout_s=100000.0)
        started = []
        p = pm.Plan(pm.Plan.SWITCH, 'B1Test', site='BACK')
        self.assertTrue(gate.submit(p, 'B1Test', 'run', False,
                                    lambda n, v: (started.append(n) or True, None)))
        time.sleep(0.02)
        gate.cancel()
        for _ in range(500):
            if not gate.busy():
                break
            time.sleep(0.01)
        self.assertEqual(started, [])
        self.assertEqual(env.reports[-1]['code'], 'cancelled')

    def test_second_start_refused_same_ignored(self):
        env = Env()
        env.navi = {'ready': False}
        gate = pm.MapGate(env, timeout_s=100000.0)
        p = pm.Plan(pm.Plan.SWITCH, 'B1Test', site='BACK')
        start = lambda n, v: (True, None)   # noqa: E731
        self.assertTrue(gate.submit(p, 'B1Test', 'run', False, start))
        self.assertFalse(gate.submit(p, 'B1Test', 'run', False, start))
        n = len(env.reports)
        other = pm.Plan(pm.Plan.SWITCH, 'Other', site='T1')
        self.assertFalse(gate.submit(other, 'Other', 'run', False, start))
        self.assertEqual(env.reports[n]['code'], 'switching')
        # The topic ends on the start that is really running.
        last = env.reports[-1]
        self.assertEqual((last['state'], last['program']), ('switching', 'B1Test'))
        gate.cancel()

    def test_program_not_runnable_after_the_load(self):
        env = Env()
        gate = pm.MapGate(env, timeout_s=10.0)
        p = pm.Plan(pm.Plan.SWITCH, 'B1Test', site='BACK')
        gate._run(p, 'B1Test', 'run', False, lambda n, v: (False, 'zone Z has no mowing paths'))
        self.assertEqual(env.reports[-1]['code'], 'program_unusable')
        self.assertIn('zone Z', env.reports[-1]['text_cs'])

    def test_worker_survives_an_exception(self):
        env = Env()

        def boom(site, legacy):
            raise RuntimeError('kaput')
        env.request_map = boom
        gate = go(env)[0]
        self.assertEqual(env.reports[-1]['state'], 'failed')
        self.assertIsNone(gate.busy())


class Texts(unittest.TestCase):
    def test_every_code_in_both_languages(self):
        for code in pm.REASONS:
            for lang in ('en', 'cs'):
                text = pm.reason_text(code, lang, program='P', site='S')
                self.assertTrue(text and '{' not in text, (code, lang))

    def test_ref(self):
        self.assertEqual(pm.parse_ref('BACK|||B1'), ('BACK', 'B1'))
        self.assertEqual(pm.parse_ref(' B1 '), (None, 'B1'))
        self.assertEqual(pm.make_ref('BACK', 'B1'), 'BACK|||B1')
        self.assertEqual(pm.make_ref(None, 'B1'), 'B1')


if __name__ == '__main__':
    unittest.main()
