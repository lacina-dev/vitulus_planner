#!/usr/bin/env python3
"""master_controller: who decides (calendar | agent), the agent's advice with a
TTL, the calendar standing in, the hard limits and the journal.

No ROS master: rospy is stubbed, the clock and the weather are the test's.
Nothing here can start a mower: the publishers only record.

Run:  python3 test/test_master_agent_mode.py
"""
import datetime as dt
import importlib.machinery
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HOME = tempfile.mkdtemp(prefix='master_agent_home_')
os.environ['HOME'] = _HOME
os.environ['TZ'] = 'Europe/Prague'
time.tzset()

import rospy                                 # noqa: E402
from std_msgs.msg import Bool, String        # noqa: E402

EVENTS = []


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        EVENTS.append((self.name, msg))


rospy.init_node = lambda *a, **kw: None
rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **kw: None
rospy.Service = lambda *a, **kw: None
rospy.get_param = lambda name, default=None: default
rospy.sleep = lambda *a, **kw: None
for _l in ('loginfo', 'logwarn', 'logerr'):
    setattr(rospy, _l, lambda *a, **kw: None)
_loader = importlib.machinery.SourceFileLoader(
    'master_under_test_agent', os.path.join(_PKG, 'nodes', 'master_controller'))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
M = importlib.util.module_from_spec(_spec)
_loader.exec_module(M)
D = dt.datetime


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
DOCK = {'ok': True, 'spent': [], 'back': [], 'asked': 0}
_real_allowed = M.dock_permit.allowed
# S8: the owners file (one source of truth) with a made-up owner.
M.dock_permit.OWNERS_FILE = os.path.join(_HOME, 'dock_permit_owners')
with open(M.dock_permit.OWNERS_FILE, 'w') as _fh:
    _fh.write('owner\n')


def _permit_allowed(now=None):
    """T18: the master reads /dock_permit/state (dock_permit node); here the
    test feeds that latched state (DOCK['ok']) and asks the real module."""
    DOCK['asked'] += 1
    M.dock_permit.on_state(json.dumps({
        'v': 1, 'stamp': time.time(), 'allowed': DOCK['ok'],
        'why': 'open' if DOCK['ok'] else 'locked',
        'active': ({'id': 'p1-1', 'by': 'owner', 'to': time.time() + 3600, 'exits': 0,
                    'max_exits': None, 'exits_left': None} if DOCK['ok'] else None),
        'homeward': None, 'enforced_by': []}))
    return _real_allowed(now)


M.dock_permit.allowed = _permit_allowed
# Nothing may be spent or given back any more: the node counts the exits.
M.dock_permit.spend = lambda rel: DOCK['spent'].append(rel) or True
M.dock_permit.give_back = lambda rel: DOCK['back'].append(rel) or True


class Base(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        self.clock = [D(2026, 6, 10, 9, 0, 0)]
        self.wx = dict(DRY)
        DOCK.update(ok=True, spent=[], back=[], asked=0)
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
        self.sleeps = []
        mc._execute_sleep = lambda duration_ms=None, charged=False: (
            self.sleeps.append((duration_ms, charged)) or True)
        return mc

    def on(self):
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE

    def agent(self, **settings):
        self.on()
        self.mc.set_mode_callback(String('agent'))
        if settings:
            self.mc._commit_settings(settings, by='test')

    def advise(self, **advice):
        advice.setdefault('id', 'a%d' % len(published('/scheduler/advice_result')))
        advice.setdefault('action', 'mow')
        if advice['action'] == 'mow':
            advice.setdefault('program', 'Mow')
        advice.setdefault('ttl_s', 3 * 3600)
        self.mc.advice_callback(String(json.dumps(advice)))
        return json.loads(published('/scheduler/advice_result')[-1])

    def tick(self, n=1):
        for _ in range(n):
            self.mc._step()

    def at(self, *a):
        self.clock[0] = D(*a)

    def starts(self):
        return published('/web_plan/program_select')

    def journal(self, kind=None):
        rows = [json.loads(x) for x in published('/scheduler/journal')][-1]['entries']
        return [(e['kind'], e['code'], e['program']) for e in rows
                if kind is None or e['kind'] == kind]

    def mode(self):
        return json.loads(published('/scheduler/mode')[-1])


class Mode(Base):
    def test_default_is_calendar_and_published(self):
        self.assertEqual(self.mc.mow_mode, 'calendar')
        self.assertEqual(self.mode()['mode'], 'calendar')
        with open(os.path.join(_HOME, '.vitulus', 'scheduler_state.json')) as f:
            self.assertEqual(json.load(f)['mow_mode'], 'calendar')

    def test_switch_persists_and_is_journalled(self):
        self.mc.set_mode_callback(String('agent'))
        self.assertEqual(self.mode()['mode'], 'agent')
        self.assertIn(('mode', 'agent', ''), self.journal('mode'))
        again = self.make()
        self.assertEqual(again.mow_mode, 'agent')
        self.mc.set_mode_callback(String(json.dumps({'mode': 'calendar', 'by': 'chat'})))
        self.assertEqual(self.make().mow_mode, 'calendar')

    def test_service_and_invalid(self):
        req = type('R', (), {'data': True})()
        self.assertTrue(self.mc.set_mode_service(req).success)
        self.assertEqual(self.mc.mow_mode, 'agent')
        self.mc.set_mode_callback(String('robot'))
        self.assertEqual(json.loads(published('/scheduler/command_result')[-1]),
                         {'cmd': 'set_mode', 'ok': False, 'code': 'invalid'})
        self.assertEqual(self.mc.mow_mode, 'agent')

    def test_enable_switches_automation_on(self):
        self.assertFalse(self.mc.automation_on)
        self.mc.set_mode_callback(String(json.dumps({'mode': 'agent', 'enable': True})))
        self.assertTrue(self.mc.automation_on)

    def test_broken_mode_in_file_is_calendar(self):
        self.mc.persistent['mow_mode'] = 'nonsense'
        self.mc._save_state()
        self.assertEqual(self.make().mow_mode, 'calendar')


class Advice(Base):
    def test_refused_in_calendar_mode(self):
        self.on()
        res = self.advise()
        self.assertEqual((res['ok'], res['code']), (False, 'mode_calendar'))
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertIn(('advice', 'rejected', 'Mow'), self.journal('advice'))

    def test_fresh_advice_starts_through_the_checks(self):
        self.agent()
        self.assertEqual(self.advise(reason='dry, 4 days')['code'], 'stored')
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        run = self.mc.persistent['run']
        self.assertEqual((run['advice'], run['event_id']), ('a0', 'advice:a0'))
        self.assertIn(('advice', 'run', 'Mow'), self.journal('advice'))
        self.assertEqual(self.mode()['advice']['state'], 'used')
        # an advice has no slot: nothing kept for ever in the slot tables
        self.assertEqual(self.mc.persistent['closed_slot_by_event'], {})

    def test_site_reference(self):
        self.agent()
        self.mc.site_programs = {'S': [{'name': 'Mow'}], 'T': [{'name': 'Far'}]}
        self.assertTrue(self.advise(program='T|||Far')['ok'])
        self.tick()
        self.assertEqual(self.starts(), ['T|||Far'])

    def test_bad_advice(self):
        self.agent()
        self.mc.site_programs = {'S': [{'name': 'Mow'}]}
        self.assertEqual(self.advise(ttl_s=10)['code'], 'ttl')
        self.assertEqual(self.advise(ttl_s=3 * 86400)['code'], 'ttl')
        self.assertEqual(self.advise(action='dance')['code'], 'action')
        self.assertEqual(self.advise(program='Nope')['code'], 'not_on_map')
        self.assertEqual(self.advise(program='')['code'], 'program')
        self.assertEqual(self.advise(to='2026-06-10T08:00')['code'], 'window')
        self.tick()
        self.assertEqual(self.starts(), [])

    def test_duplicate_id_is_ignored(self):
        self.agent()
        self.advise(id='x')
        self.assertEqual(self.advise(id='x')['code'], 'duplicate')

    def test_window_in_the_future_waits(self):
        self.agent()
        self.advise(**{'from': '2026-06-10T14:00', 'to': '2026-06-10T17:00', 'ttl_s': 12 * 3600})
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'agent_wait_window')
        self.assertTrue(self.sleeps)            # sleeps until shortly before 14:00
        self.at(2026, 6, 10, 14, 1)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_window_passed_unstarted_is_missed(self):
        self.agent()
        self.advise(**{'to': '2026-06-10T10:00'})
        self.mc.smach_status = 'Stopped'      # a person's STOP: nothing starts
        self.tick()
        self.at(2026, 6, 10, 10, 30)
        self.mc.smach_status = 'Ready'
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertIn(('advice', 'missed', 'Mow'), self.journal('advice'))

    def test_expired_advice_falls_back_to_the_calendar(self):
        self.agent()
        self.mc.calendar = [daily('11:00')]
        self.advise(ttl_s=3600)
        self.mc.smach_status = 'Stopped'
        self.tick()
        self.at(2026, 6, 10, 11, 0, 30)
        self.mc.smach_status = 'Ready'
        self.tick()
        self.assertIn(('advice', 'expired', 'Mow'), self.journal('advice'))
        self.assertIn(('advice', 'calendar', 'Mow'), self.journal('advice'))
        self.assertEqual(self.starts(), ['Mow'])

    def test_no_fallback_without_advice_nothing_starts(self):
        self.agent(agent_fallback_calendar=False)
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'agent_no_advice')
        self.assertIn(('advice', 'none', ''), self.journal('advice'))

    def test_skip_closes_the_calendar_slot(self):
        self.at(2026, 6, 10, 8, 0)
        self.agent()
        self.mc.calendar = [daily('09:00')]
        self.advise(action='skip', until='2026-06-10T18:00', ttl_s=12 * 3600,
                    reason='rain tonight')
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertIn(('skipped', 'advice_skip', 'Mow'), self.journal('skipped'))
        self.assertEqual(self.mc._code, 'agent_skip')
        # the skip is over: the next day's slot is the calendar's again
        self.at(2026, 6, 11, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_back_to_calendar_drops_an_unstarted_advice_run(self):
        self.agent()
        self.mc.battery_capacity = 10         # the run opens and waits for charge
        self.advise()
        self.mc._start_blocker = lambda now: None
        self.tick()
        self.assertIsNotNone(self.mc.persistent['run'])
        self.mc.set_mode_callback(String('calendar'))
        self.assertIsNone(self.mc.persistent['run'])
        self.assertIn(('advice', 'dropped', 'Mow'), self.journal('advice'))


class Limits(Base):
    def test_rain_three_hours_holds_the_advice(self):
        self.agent()
        self.wx = {'hours_since_rain': 1.0, 'rain_last_h': False, 'temp_now': 15.0}
        self.advise()
        self.tick(3)
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'agent_blocked')
        # the UI writes the limit with its numbers: they go along with the code
        self.assertEqual(self.mc._code_detail.get('why'), 'limit:limit_rain')
        self.assertEqual((self.mc._code_detail.get('hours_since_rain'), self.mc._code_detail.get('need_h')), (1.0, 3.0))
        waits = [e for e in self.journal('advice') if e[1] == 'waiting']
        self.assertEqual(len(waits), 1)            # once, not every look
        self.wx = dict(DRY)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_unfinished_run_is_not_recorded(self):
        self.agent()
        self.mc.node_start_time = 0
        self.mc.mission_started_callback(String('Mow'))
        self.mc.stop_reason_callback(String('interrupted:weather'))
        self.assertIsNone(self.mc._last_mowed('Mow'))
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_stale_weather_holds_the_agent_not_the_calendar(self):
        self.agent()
        self.wx = dict(DRY, stale={'forecast': True, 'radar': True})
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mode()['limit_now']['code'], 'limit_weather_unknown')
        self.mc.set_mode_callback(String('calendar'))
        self.mc._commit_settings({'limits_in_calendar': True}, by='test')
        self.assertIsNone(self.mc._limit_verdict(self.clock[0], 'Mow', fresh=False))

    def test_cold(self):
        self.agent()
        self.wx = {'hours_since_rain': 50.0, 'temp_now': 3.5}
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mode()['limit_now']['code'], 'limit_cold')

    def test_night_limit_is_the_window(self):
        # One window (owner 2026-11-08): the agent's night limit is the allowed
        # window of the calendar, with its times; no hours of its own.
        self.at(2026, 6, 10, 6, 30)               # window 07-21 closed
        self.agent()
        self.advise(ttl_s=6 * 3600)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mode()['limit_now']['code'], 'limit_night')
        self.assertEqual(self.mode()['limit_now']['vars'],
                         {'start': '07:00', 'end': '21:00', 'cause': 'outside_window',
                          'min_left_min': 30.0})
        self.assertEqual(self.mc._code, 'agent_blocked')
        self.assertEqual(self.mc._code_detail['why'], 'limit:limit_night')
        self.assertIn(('advice', 'waiting', 'Mow'), self.journal('advice'))
        self.assertEqual(self.mc.persistent['last_decision']['vars']['why'], 'limit:limit_night')
        self.assertEqual(self.mc.persistent['last_decision']['vars']['start'], '07:00')
        self.at(2026, 6, 10, 7, 1)                # 07:30 was the old agent-only limit: gone
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_night_limit_sleeps_until_the_window(self):
        self.at(2026, 6, 10, 5, 0)
        self.agent()
        self.advise(ttl_s=6 * 3600)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertTrue(self.sleeps)
        ms = self.sleeps[-1][0]
        self.assertEqual(ms, int((2 * 3600 - self.mc.wake_lead_s) * 1000))   # to 07:00

    def test_night_limit_too_little_left_like_the_calendar(self):
        self.at(2026, 6, 10, 20, 45)              # 15 min of 07-21 left, 30 needed
        self.agent()
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mode()['limit_now']['vars']['cause'], 'too_little_left')

    def test_night_limit_over_midnight(self):
        self.agent(window_start='22:00', window_end='05:00', window_min_left_min=10.0)
        self.at(2026, 6, 10, 12, 0)
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mode()['limit_now']['vars']['start'], '22:00')
        self.at(2026, 6, 11, 1, 0)
        self.advise(ttl_s=3600)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_window_off_time_does_not_hold_the_agent(self):
        self.at(2026, 6, 10, 23, 30)
        self.agent(window_enabled=False)
        self.assertIsNone(self.mc._limit_verdict(self.clock[0], 'Mow'))
        lim = self.mc._mode_view(self.clock[0])['limits']
        self.assertEqual(lim['window']['enabled'], False)
        self.assertIsNone(lim['day_start'])
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_mode_view_carries_the_window(self):
        self.agent(window_start='06:30', window_end='20:15')
        lim = self.mc._mode_view(self.clock[0])['limits']
        self.assertEqual(lim['window'], {'enabled': True, 'start': '06:30', 'end': '20:15',
                                         'min_left_min': 30.0})
        self.assertEqual((lim['day_start'], lim['day_end']), ('06:30', '20:15'))
        self.assertNotIn('limit_day_start', self.mc.settings)

    def test_agent_sleep_cap_is_the_window_start(self):
        self.agent(window_start='06:00')
        self.at(2026, 6, 10, 22, 0)
        self.mc._sleep_for('test', 3 * 86400)
        ms = self.sleeps[-1][0]
        self.assertEqual(ms, int((8 * 3600 - self.mc.wake_lead_s) * 1000))   # to 06:00

    def test_unknown_weather_does_not_refuse(self):
        self.agent()
        self.wx = {}                      # no weather at all: the agent does not start
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), [])
        # fresh data with gaps (no rain on record, no temperature): no refusal
        self.wx = {'stale': {'forecast': False, 'radar': True}}
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_zone_gap(self):
        self.at(2026, 6, 9, 10, 0)
        self.agent()
        self.mc.node_start_time = 0
        for name in ('Back', 'Mow'):                        # C; A and B
            self.mc.mission_started_callback(String(name))
            self.mc.stop_reason_callback(String('succeeded'))
            self.mc.stop_reason_callback(String(''))
        self.at(2026, 6, 10, 9, 0)
        self.assertEqual(self.mc._last_mowed('Mow')[0], 'Mow')
        self.advise(program='Mow', ttl_s=47 * 3600)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'agent_blocked')
        self.at(2026, 6, 11, 10, 1)               # two days on
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_zone_gap_skips_a_calendar_slot_in_agent_mode(self):
        self.at(2026, 6, 10, 8, 0)
        self.agent()
        self.mc.persistent['last_mow'] = {'zones': {'S|A': '2026-06-09T18:00:00'},
                                          'programs': {}}
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertIsNone(self.mc.persistent['run'])
        self.assertIn(('skipped', 'limit_zone_gap', 'Mow'), self.journal('skipped'))

    def test_calendar_mode_unchanged_by_default(self):
        self.at(2026, 6, 10, 7, 0)
        self.on()
        self.wx = {'hours_since_rain': 0.5, 'temp_now': 2.0}
        self.mc.calendar = [daily('07:30')]
        self.at(2026, 6, 10, 7, 30, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_calendar_mode_with_limits_on(self):
        self.at(2026, 6, 10, 8, 0)
        self.on()
        self.mc._commit_settings({'limits_in_calendar': True}, by='test')
        self.wx = {'hours_since_rain': 0.5, 'temp_now': 15.0}
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'limit_rain')
        self.assertIn(('waited', 'limit_rain', 'Mow'), self.journal('waited'))

    def test_run_next_by_hand_is_not_limited(self):
        self.agent()
        self.wx = {'hours_since_rain': 0.5, 'temp_now': 15.0}
        self.mc.calendar = [daily('15:00')]
        self.mc.weather_msg = None
        self.mc._run_next()
        self.assertEqual(self.starts(), ['Mow'])

    def test_resume_ignores_the_zone_gap(self):
        self.agent()
        self.mc.persistent['last_mow'] = {'zones': {}, 'programs': {'Mow': '2026-06-10T08:00:00'}}
        self.assertIsNone(self.mc._limit_verdict(self.clock[0], 'Mow', fresh=False))
        self.assertEqual(self.mc._limit_verdict(self.clock[0], 'Mow')[0], 'limit_zone_gap')


class Dock(Base):
    def test_no_permission_holds_the_advice(self):
        self.agent()
        DOCK['ok'] = False
        self.advise()
        self.tick(3)
        self.assertEqual(self.starts(), [])
        self.assertEqual(self.mc._code, 'waiting_dock')
        waits = [e for e in self.journal('advice') if e[1] == 'waiting']
        self.assertEqual(len(waits), 1)
        self.assertEqual(self.mode()['advice']['state'], 'pending')   # not used up
        DOCK['ok'] = True
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.assertEqual(DOCK['spent'], [])       # T18: the node counts the exit

    def test_rejected_start_spends_nothing(self):
        """T18: nothing is spent before the start, so a start that did not
        leave the dock has nothing to give back (the node never counted it)."""
        self.agent()
        self.advise()
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        mc = self.mc
        mc._mowing_grace_until = 0
        mc._mission_seen = True
        mc.stop_reason = 'rejected:battery'
        self.tick()
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.tick()
        self.assertEqual((DOCK['spent'], DOCK['back']), ([], []))
        self.assertNotIn('dock_spent', mc.persistent.get('run') or {})

    def test_calendar_runs_do_not_ask_the_lock(self):
        self.at(2026, 6, 10, 8, 0)
        self.agent()
        DOCK['ok'] = False
        self.mc.calendar = [daily('09:00')]
        self.at(2026, 6, 10, 9, 0, 30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_switch_to_calendar_wins_over_the_fsm(self):
        self.agent()
        self.advise()
        aid = self.mc.persistent['advice']['id']
        self.mc.set_mode_callback(String('calendar'))
        self.assertFalse(self.mc._advice_mark(aid, 'used'))
        self.mc.mow_mode = 'agent'          # a tick that read the old mode
        self.assertFalse(self.mc._advice_mark(aid, 'used'))   # dropped stays dropped
        self.tick()
        self.assertEqual(self.starts(), [])

    def test_fresh_advice_is_a_copy(self):
        self.agent()
        self.advise()
        got = self.mc._advice_fresh(self.clock[0])
        got['state'] = 'used'
        self.assertEqual(self.mc.persistent['advice']['state'], 'pending')


class Sleep(Base):
    def test_agent_mode_never_sleeps_past_the_next_morning(self):
        self.agent()
        self.mc.calendar = [{'id': 'w', 'program_name': 'Mow', 'enabled': True,
                             'recurrence': 'weekly', 'days_of_week': [0], 'start_time': '09:00'}]
        self.mc.wake_lead_s = 0
        self.at(2026, 6, 10, 12, 0)                   # Wednesday
        self.tick()
        self.assertTrue(self.sleeps)
        ms = self.sleeps[-1][0]
        self.assertLessEqual(ms / 1000.0, 20 * 3600 + 1)   # to 08:00 tomorrow


class ReasonCode(Base):
    """T22 (N6): the advice's reason also as a code for the UI; never decides."""

    def test_code_and_vars_go_to_mode_and_journal(self):
        self.agent()
        res = self.advise(reason='pršelo před 1.0 h', reason_code='rain_recent',
                          reason_vars={'h': 1.0, 'need': 3})
        self.assertEqual(res['code'], 'stored')
        adv = self.mode()['advice']
        self.assertEqual((adv['reason'], adv['reason_code'], adv['reason_vars']),
                         ('pršelo před 1.0 h', 'rain_recent', {'h': 1.0, 'need': 3}))
        rows = [json.loads(x) for x in published('/scheduler/journal')][-1]['entries']
        got = [e['vars'] for e in rows if (e['kind'], e['code']) == ('advice', 'received')]
        self.assertEqual((got[-1]['reason_code'], got[-1]['reason_vars']),
                         ('rain_recent', {'h': 1.0, 'need': 3}))
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])        # the code changes nothing
        self.assertEqual(self.mode()['last_decision']['vars']['reason_code'], 'rain_recent')

    def test_old_agent_and_bad_codes(self):
        self.agent()
        self.advise(reason='plain words')
        self.assertNotIn('reason_code', self.mode()['advice'])
        for bad in ('Rain', '1x', 'a b', 'x' * 41, 7, ''):
            self.advise(reason='r', reason_code=bad)
            self.assertNotIn('reason_code', self.mode()['advice'], bad)
        self.advise(reason='r', reason_code='mow', reason_vars={'s': 'x' * 2000})
        self.assertEqual(self.mode()['advice']['reason_vars'], {})
        self.advise(reason='r', reason_code='mow', reason_vars=['not', 'a', 'dict'])
        self.assertEqual(self.mode()['advice']['reason_vars'], {})


class WeatherCache(Base):
    """T22 (N8): summary() is read at most every WEATHER_CACHE_S, timed, and
    read again when the clock went back."""

    def test_cache_cost_and_clock_back(self):
        calls = []
        saved = M.weather_history
        M.weather_history = type('H', (), {'summary': staticmethod(
            lambda t: calls.append(t) or dict(DRY))})
        try:
            mc = self.make()
            del mc._weather_summary                  # the real, cached one
            mc._wx_t = -1e12
            del calls[:]                             # the constructor read it on the real clock
            mc._weather_summary()
            mc._weather_summary()
            self.assertEqual(len(calls), 1)
            self.assertIsNotNone(mc._wx_cost_s)
            self.at(2026, 6, 10, 9, 4, 59)
            mc._weather_summary()
            self.assertEqual(len(calls), 1)
            self.at(2026, 6, 10, 9, 5, 0)
            mc._weather_summary()
            self.assertEqual(len(calls), 2)
            self.at(2026, 6, 10, 8, 0)               # the clock was fixed backwards
            self.assertEqual(mc._weather_summary(), DRY)
            self.assertEqual(len(calls), 3)
        finally:
            M.weather_history = saved


if __name__ == '__main__':
    unittest.main()
