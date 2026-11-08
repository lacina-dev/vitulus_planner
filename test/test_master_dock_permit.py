#!/usr/bin/env python3
"""master_controller + uzel povolení doku `dock_permit` (T18, etapa 3).

Master čte `/dock_permit/state` přímo (bez agenta).  Tady se krmí týž JSON,
jaký uzel publikuje, skutečnému modulu `node_planner.dock_permit` a měří se:

* rada agenta vyjede jen při platném okně; bez stavu / se starým stavem ne
  (fail-closed jen pro agenta);
* kalendář se povolení neptá vůbec — ani když uzel neběží (D37);
* K1: povolení se čte jen před výjezdem; sečení venku nic nezastaví;
* homeward (D38): konec okna s robotem venku → dosekat cestu (max deadline),
  pak domů; běh zůstane přerušený a naváže až s novým povolením; nové okno
  homeward zruší; Q2 jedním parametrem (`dock_permit_homeward`).

No ROS master: rospy is stubbed, the clock is the test's.  Nothing here can
start a mower: the publishers only record.

Run:  python3 test/test_master_dock_permit.py
"""
import datetime as dt
import importlib.machinery
import importlib.util
import json
import os
import shutil
import tempfile
import time
import unittest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HOME = tempfile.mkdtemp(prefix='master_permit_home_')
os.environ['HOME'] = _HOME
os.environ['TZ'] = 'Europe/Prague'
time.tzset()

import rospy                                 # noqa: E402
from std_msgs.msg import String              # noqa: E402

EVENTS = []
PARAMS = {}


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        EVENTS.append((self.name, msg))


rospy.init_node = lambda *a, **kw: None
rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **kw: None
rospy.Service = lambda *a, **kw: None
rospy.get_param = lambda name, default=None: PARAMS.get(name, default)
rospy.sleep = lambda *a, **kw: None
for _l in ('loginfo', 'logwarn', 'logerr'):
    setattr(rospy, _l, lambda *a, **kw: None)
_loader = importlib.machinery.SourceFileLoader(
    'master_under_test_permit', os.path.join(_PKG, 'nodes', 'master_controller'))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
M = importlib.util.module_from_spec(_spec)
_loader.exec_module(M)
P = M.dock_permit
D = dt.datetime
OWNERS = os.path.join(_HOME, 'dock_permit_owners')    # S8: one source of truth


def owners(names=('owner',)):
    if names is None:
        if os.path.exists(OWNERS):
            os.remove(OWNERS)
    else:
        with open(OWNERS, 'w') as fh:
            fh.write('# made-up owner\n' + ''.join('%s\n' % n for n in names))
    P.OWNERS_FILE = OWNERS


owners()


def published(topic):
    return [m.data for t, m in EVENTS if t == topic]


def daily(t, eid='ev1', prog='Mow'):
    return {'id': eid, 'program_name': prog, 'enabled': True, 'recurrence': 'daily',
            'start_time': t}


class _Zone(object):
    def __init__(self, name):
        self.name = name


def program(name, zones, site='S'):
    return type('P', (), {'name': name, 'last_result': '', 'map_name': site,
                          'zone_list': [_Zone(z) for z in zones]})()


DRY = {'hours_since_rain': 30.0, 'rain_last_h': False, 'temp_now': 18.0,
       'stale': {'forecast': False, 'radar': False}}


def state(allowed=True, why=None, homeward=None, stamp=None, by='owner'):
    """/dock_permit/state, jak ho publikuje uzel (kontrakt §4)."""
    now = time.time() if stamp is None else stamp
    return json.dumps({
        'v': 1, 'stamp': now, 'allowed': allowed,
        'why': why or ('open' if allowed else 'locked'),
        'active': ({'id': 'p1-1', 'by': by, 'from': now - 60, 'to': now + 3600,
                    'max_exits': None, 'exits': 0, 'exits_left': None}
                   if allowed else None),
        'homeward': homeward, 'dock': 'in', 'clock_ok': True, 'enforced_by': []})


def hw(since=None, chunk=300.0):
    since = time.time() if since is None else since
    return {'since': since, 'deadline': since + chunk, 'finish_chunk_s': chunk,
            'window': 'p1-1'}


class Base(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        PARAMS.clear()
        P.VIEW = P.PermitView()               # no state at all: node absent
        owners()
        self.clock = [D(2026, 6, 10, 9, 0, 0)]
        self.wx = dict(DRY)
        self.mc = self.make()

    def make(self):
        mc = M.MasterController()
        mc._now = lambda: self.clock[0]
        mc._t = lambda: time.mktime(self.clock[0].timetuple())
        mc.boot_grace_period_s = 0
        mc.check_interval_s = 0
        mc.startup_delay_s = 0
        mc.programs_loaded = True
        mc.program_list = [program('Mow', ['A', 'B']), program('Back', ['C'])]
        mc.is_in_dock = True
        mc.battery_capacity = 90
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.dock_status = 0
        mc._weather_summary = lambda: self.wx
        mc._execute_sleep = lambda duration_ms=None, charged=False: True
        return mc

    def permit(self, *a, **kw):
        P.on_state(state(*a, **kw))

    def agent(self):
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE
        self.mc.set_mode_callback(String('agent'))

    def advise(self, **advice):
        advice.setdefault('id', 'a%d' % len(published('/scheduler/advice_result')))
        advice.setdefault('action', 'mow')
        advice.setdefault('program', 'Mow')
        advice.setdefault('ttl_s', 3 * 3600)
        self.mc.advice_callback(String(json.dumps(advice)))
        return json.loads(published('/scheduler/advice_result')[-1])

    def tick(self, n=1):
        for _ in range(n):
            self.mc._step()

    def at(self, *a):
        self.clock[0] = D(*a)

    def later(self, s):
        self.clock[0] = self.clock[0] + dt.timedelta(seconds=s)

    def starts(self):
        return published('/web_plan/program_select')

    def resumes(self):
        return published('/web_plan/program_select_resume')

    def journal(self, kind=None):
        rows = [json.loads(x) for x in published('/scheduler/journal')][-1]['entries']
        return [(e['kind'], e['code'], e['program']) for e in rows
                if kind is None or e['kind'] == kind]

    def out_mowing(self):
        """The mission has left the dock and mows."""
        mc = self.mc
        self.assertEqual(mc.state, mc.STATE_MOWING)
        mc._mowing_grace_until = 0
        mc._mission_seen = True
        mc.smach_status = 'Mowing'
        mc.mission_busy = True
        mc.is_in_dock = False
        mc.mission_started_name = 'Mow'

    def home_with(self, stop_reason):
        mc = self.mc
        mc.is_in_dock = True
        mc.mission_busy = False
        mc.smach_status = 'Ready'
        mc.stop_reason = stop_reason


class View(unittest.TestCase):
    def setUp(self):
        owners()

    def test_fail_closed_and_fresh(self):
        t = [100.0]
        v = P.PermitView(mono=lambda: t[0])
        self.assertEqual(v.allowed()[:2], (False, 'no_state'))
        v.on_state('{nesmysl')
        v.on_state(json.dumps({'no': 'allowed'}))
        self.assertEqual(v.allowed()[:2], (False, 'no_state'))
        v.on_state(state(True))
        self.assertEqual(v.allowed()[:2], (True, 'open'))
        t[0] += 29
        self.assertTrue(v.allowed()[0])
        t[0] += 2                                   # 31 s: stale
        self.assertEqual(v.allowed()[:2], (False, 'stale'))
        v.on_state(state(False, 'exits_used'))
        self.assertEqual(v.allowed()[:2], (False, 'exits_used'))

    def test_homeward_counts_down_on_this_machine(self):
        t = [100.0]
        v = P.PermitView(mono=lambda: t[0])
        now = time.time()
        v.on_state(state(False, homeward=hw(since=now - 100), stamp=now))
        got = v.homeward()
        self.assertAlmostEqual(got['left_s'], 200.0, delta=0.5)
        t[0] += 20
        self.assertAlmostEqual(v.homeward()['left_s'], 180.0, delta=0.5)
        t[0] += 20                                   # stale state: no homeward
        self.assertIsNone(v.homeward())
        v.on_state(state(False, homeward=None))
        self.assertIsNone(v.homeward())

    def test_owners_file_rules_like_the_node(self):
        """utf-8-sig (a BOM from a Windows editor), CRLF, comments, case;
        an undecodable file is unreadable (fail-closed)."""
        for data, want in ((b'\xef\xbb\xbfOwner\r\n# x\n', (['owner'], 'ok')),
                           (b'# only a comment\n', ([], 'empty')),
                           (b'own\xe9r\n', ([], 'unreadable'))):
            with open(OWNERS, 'wb') as fh:
                fh.write(data)
            self.assertEqual(P.read_owners(OWNERS), want)
        owners()

    def test_q2_is_one_parameter(self):
        self.assertEqual(P.homeward_for(None), 'agent')
        self.assertEqual(P.homeward_for('ALL'), 'all')
        self.assertEqual(P.homeward_for('nesmysl'), 'agent')
        self.assertTrue(P.spend({}) and P.give_back({}))


class Advice(Base):
    def test_no_node_no_advice_run(self):
        """Fail-closed for the agent: the node absent = no permission."""
        self.agent()
        self.advise()
        self.tick(3)
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'waiting_dock')
        self.permit(True)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_locked_and_stale_hold_it(self):
        self.agent()
        self.permit(False, 'exits_used')
        self.advise()
        self.tick(2)
        self.assertEqual(self.starts(), [])
        checks = dict((c['key'], c) for c in self.mc._start_checks(self.clock[0]))
        self.assertEqual((checks['permit']['state'], checks['permit']['code']),
                         ('wait', 'exits_used'))
        P.VIEW.mono = lambda: time.monotonic() + 3600     # the node fell silent
        self.permit(True)
        P.VIEW._rx -= 3600 * 2
        self.tick()
        self.assertEqual(self.starts(), [])

    def test_the_permit_is_asked_right_before_the_start(self):
        """Allowed at the advice, locked a moment later: no start."""
        self.agent()
        self.permit(True)
        self.advise()
        run_ok = self.mc._advice_run_may_go
        def lock_then(run):
            self.permit(False)
            return run_ok(run)
        self.mc._advice_run_may_go = lock_then
        self.tick()
        self.assertEqual(self.starts(), [])

    def test_mowing_out_is_never_stopped_by_the_permit(self):
        """K1: allowed:false while mowing (exits used, window over) is not
        a stop; only homeward sends it home."""
        self.agent()
        self.permit(True)
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.out_mowing()
        self.permit(False, 'exits_used')
        self.tick(5)
        self.later(3600)
        self.permit(False, 'locked')
        self.tick(5)
        self.assertEqual(self.mc.state, self.mc.STATE_MOWING)
        self.assertEqual(published('/mower_smach/return_home'), [])
        self.assertEqual(published('/mower_smach/stop'), [])
        self.assertNotIn(True, [m for m in published('/mower_smach/finish_after_path')])


class Owners(Base):
    """S7/S8: only a window signed by an owner from the owners file counts,
    for a fresh advice and for resuming an interrupted advice run."""

    def test_no_owners_file_no_advice_run(self):
        owners(None)
        self.agent()
        self.permit(True)
        self.advise()
        self.tick(2)
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'waiting_dock')
        checks = dict((c['key'], c) for c in self.mc._start_checks(self.clock[0]))
        self.assertEqual(checks['permit']['code'], 'no_owners_missing')
        owners()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_a_window_of_someone_else_does_not_count(self):
        self.agent()
        self.permit(True, by='host')
        self.advise()
        self.tick(2)
        self.assertEqual(self.starts(), [])
        self.assertEqual(P.allowed()[:2], (False, 'not_owner'))

    def test_resume_after_homeward_needs_an_owner_window(self):
        self.agent()
        self.permit(True)
        self.advise()
        self.tick()
        self.out_mowing()
        self.permit(False, homeward=hw(chunk=0.0))
        self.tick(2)
        self.home_with('interrupted:dock_permit')
        self.permit(False)
        self.tick(3)
        self.permit(True, by='host')
        self.tick(2)
        self.assertEqual(self.resumes(), [])
        self.permit(True)
        self.tick(2)
        self.assertEqual(self.resumes(), ['Mow'])

    def test_the_calendar_needs_no_owners_file(self):
        owners(None)
        self.at(2026, 6, 10, 8, 0)
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])


class Calendar(Base):
    def test_calendar_never_asks_even_without_the_node(self):
        """D37: today's behaviour, also with no /dock_permit/state at all."""
        self.at(2026, 6, 10, 8, 0)
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        keys = [c['key'] for c in self.mc._start_checks(self.clock[0])]
        self.assertNotIn('permit', keys)

    def test_calendar_run_in_agent_mode_without_permit(self):
        self.at(2026, 6, 10, 8, 0)
        self.agent()
        self.permit(False)
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_calendar_run_out_ignores_homeward_by_default(self):
        """Q2 default 'agent': a calendar run finishes as before."""
        self.at(2026, 6, 10, 8, 0)
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.out_mowing()
        self.permit(False, homeward=hw())
        self.tick(3)
        self.later(400)
        self.tick(3)
        self.assertEqual(published('/mower_smach/return_home'), [])
        self.assertNotIn(True, published('/mower_smach/finish_after_path'))

    def test_run_next_by_hand_does_not_ask_the_permit(self):
        self.agent()
        self.mc.calendar = [daily('15:00')]
        self.advise()                     # pending advice, no permit
        self.mc._run_next()
        res = json.loads(published('/scheduler/command_result')[-1])
        self.assertNotIn('permit', str(res.get('code') or res))


class Homeward(Base):
    def go_out(self):
        self.agent()
        self.permit(True)
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.out_mowing()

    def test_finish_the_path_then_home_and_wait_for_a_permit(self):
        self.go_out()
        self.permit(False, homeward=hw())
        self.tick()
        self.assertEqual(published('/mower_smach/finish_after_path')[-1], True)
        self.assertIn(('dock_permit', 'homeward', 'Mow'), self.journal('dock_permit'))
        self.later(200)
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), [])   # still finishing
        self.later(110)
        self.permit(False, homeward=hw(since=time.time() - 310))
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), ['dock_permit'])
        self.tick(3)
        self.assertEqual(published('/mower_smach/return_home'), ['dock_permit'])  # once
        # home: the run stays interrupted, it waits for a new permit
        self.home_with('interrupted:dock_permit')
        self.permit(False)
        self.tick(3)
        run = self.mc._run()
        self.assertIsNotNone(run)
        self.assertEqual(run.get('reason'), 'dock_permit')
        self.assertEqual(self.mc._code, 'waiting_dock')
        self.assertEqual(self.resumes(), [])
        self.permit(True)
        self.tick(2)
        self.assertEqual(self.resumes(), ['Mow'])

    def test_path_end_reports_window_but_counts_as_permit(self):
        """mower_smach says interrupted:window at the path boundary."""
        self.go_out()
        self.permit(False, homeward=hw())
        self.tick()
        self.home_with('interrupted:window')
        self.permit(False)
        self.tick(3)
        self.assertEqual((self.mc._run() or {}).get('reason'), 'dock_permit')
        self.assertEqual(self.mc._code, 'waiting_dock')

    def test_a_new_window_cancels_it(self):
        self.go_out()
        self.permit(False, homeward=hw())
        self.tick()
        self.assertEqual(published('/mower_smach/finish_after_path')[-1], True)
        self.permit(True)                    # node: homeward_cancel
        self.tick()
        self.assertEqual(published('/mower_smach/finish_after_path')[-1], False)
        self.assertIn(('dock_permit', 'homeward_cancel', 'Mow'), self.journal('dock_permit'))
        self.later(400)
        self.tick(2)
        self.assertEqual(published('/mower_smach/return_home'), [])

    def test_a_silent_node_is_not_a_cancel(self):
        """The node falls silent mid-homeward (state older than 30 s): keep
        counting to the stored deadline, then home; no homeward_cancel."""
        self.go_out()
        self.permit(False, homeward=hw())
        self.tick()
        self.assertEqual(published('/mower_smach/finish_after_path'), [True])
        P.VIEW._rx -= 45                                  # silent for 45 s
        self.later(45)
        self.tick()
        self.assertNotIn(False, published('/mower_smach/finish_after_path'))
        self.assertNotIn(('dock_permit', 'homeward_cancel', 'Mow'),
                         self.journal('dock_permit'))
        self.assertEqual(published('/mower_smach/return_home'), [])
        self.later(300)
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), ['dock_permit'])
        P.VIEW = P.PermitView()                           # no state at all
        self.later(30)
        self.tick(2)
        self.assertEqual(published('/mower_smach/return_home'), ['dock_permit'])

    def test_q2_none_never_sends_home(self):
        PARAMS['~dock_permit_homeward'] = 'none'
        self.mc = self.make()
        self.go_out()
        self.permit(False, homeward=hw())
        self.tick()
        self.later(400)
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), [])

    def test_q2_all_sends_a_calendar_run_home_and_it_waits_for_a_permit(self):
        PARAMS['~dock_permit_homeward'] = 'all'
        self.mc = self.make()
        self.at(2026, 6, 10, 8, 0)
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.out_mowing()
        self.permit(False, homeward=hw(chunk=0.0))
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), ['dock_permit'])
        self.home_with('interrupted:dock_permit')
        self.permit(False)
        self.tick(3)
        self.assertEqual(self.mc._code, 'waiting_dock')
        self.assertEqual(self.resumes(), [])


if __name__ == '__main__':
    unittest.main()
