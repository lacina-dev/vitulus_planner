#!/usr/bin/env python3
"""master_controller: slots, catch-up, the switch, persistence, DST, caps.

No ROS master: rospy is stubbed, the clock is the test's. The full cycle against
the real mission is in calendar_sim/run_sim.py.

Run:  python3 test/test_master_schedule.py
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
_HOME = tempfile.mkdtemp(prefix='master_sched_home_')
os.environ['HOME'] = _HOME
os.environ['TZ'] = 'Europe/Prague'
time.tzset()

import rospy                                 # noqa: E402
from std_msgs.msg import Bool                # noqa: E402

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
    'master_under_test2', os.path.join(_PKG, 'nodes', 'master_controller'))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
M = importlib.util.module_from_spec(_spec)
_loader.exec_module(M)
S = M.MowerScheduler
D = dt.datetime


def published(topic):
    return [m.data for t, m in EVENTS if t == topic]


def daily(t, eid='ev1', prog='Mow'):
    return {'id': eid, 'program_name': prog, 'enabled': True, 'recurrence': 'daily',
            'start_time': t}


class Base(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        self.clock = [D(2026, 6, 10, 7, 0, 0)]
        self.mc = self.make()

    def make(self):
        mc = M.MasterController()
        mc._now = lambda: self.clock[0]
        mc._t = lambda: time.mktime(self.clock[0].timetuple())
        mc.boot_grace_period_s = 0
        mc.check_interval_s = 0
        mc.startup_delay_s = 0
        mc.programs_loaded = True
        mc.program_list = [type('P', (), {'name': 'Mow', 'last_result': ''})()]
        mc.is_in_dock = True
        mc.battery_capacity = 90
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.dock_status = 0
        self.sleeps = []
        mc._execute_sleep = lambda duration_ms=None, charged=False: (
            self.sleeps.append((duration_ms, charged)) or True)
        return mc

    def on(self):
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE

    def cal(self, *ev):
        self.mc.calendar = list(ev)

    def tick(self, n=1):
        for _ in range(n):
            self.mc._step()

    def at(self, *a):
        self.clock[0] = D(*a)

    def starts(self):
        return published('/web_plan/program_select')

    def resumes(self):
        return published('/web_plan/program_select_resume')

    def run_started(self):
        mc = self.mc
        mc._mowing_grace_until = 0
        mc.smach_status = 'Mowing started'
        mc.mission_busy = True
        mc.active_program_name = 'Mow'
        mc.mission_started_name = 'Mow'
        self.tick()

    def end(self, reason):
        mc = self.mc
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.stop_reason = reason
        mc.active_program_name = ' '
        self.tick()


class Switch(Base):
    def test_default_off_and_no_action(self):
        mc = self.mc
        self.cal(daily('08:00'))
        self.assertFalse(mc.automation_on)
        mc.state = mc.STATE_INIT
        self.at(2026, 6, 10, 8, 0, 30)
        self.tick(3)
        self.assertEqual(mc.state, mc.STATE_DISABLED)
        self.assertEqual((self.starts(), self.sleeps), ([], []))
        self.at(2026, 6, 10, 23, 0, 0)         # outside the window: still no sleep
        self.tick(3)
        self.assertEqual(self.sleeps, [])

    def test_persisted_and_old_state_file_is_off(self):
        self.on()
        self.assertTrue(self.make().automation_on)
        path = self.mc.state_file
        with open(path) as f:
            d = json.load(f)
        del d['automation_enabled']
        with open(path, 'w') as f:
            json.dump(d, f)
        os.remove(path + '.prev') if os.path.exists(path + '.prev') else None
        self.assertFalse(self.make().automation_on)

    def test_on_releases_hold_and_does_not_catch_up(self):
        self.mc.is_inhibited = True
        self.cal(daily('06:50'))
        self.on()
        self.assertFalse(self.mc.is_inhibited)
        self.assertEqual(published('/scheduler/inhibit_status')[-1], False)
        self.tick(2)
        self.assertEqual(self.starts(), [])

    def test_off_while_mowing_leaves_it_alone_and_forgets_the_run(self):
        self.on()
        self.cal(daily('07:00'))
        self.at(2026, 6, 10, 7, 0, 0)
        self.mc.persistent['scan_from'] = '2026-06-10T06:59:00'
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.run_started()
        self.mc.set_automation(False)
        self.tick()
        self.assertEqual(self.mc.state, self.mc.STATE_DISABLED)
        self.assertIsNone(self.mc.persistent['run'])
        self.assertEqual(published('/mower_smach/stop'), [])
        self.end('succeeded')
        self.assertEqual(self.sleeps, [])


class Slots(Base):
    def setUp(self):
        Base.setUp(self)
        self.cal(daily('08:00'))
        self.on()

    def test_start_once_last_run_only_after_real_start(self):
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.assertEqual(self.mc.persistent['last_run_by_event'], {})
        self.run_started()
        self.assertEqual(self.mc.persistent['last_run_by_event'], {'ev1': '2026-06-10'})
        self.end('succeeded')
        self.tick(3)
        self.assertEqual(self.starts(), ['Mow'])

    def test_rejected_slot_keeps_last_run_empty_and_retries_fresh(self):
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.mc._mowing_grace_until = 0
        self.mc.smach_status = 'Rejected: rain'
        self.mc.stop_reason = 'rejected:rain'
        self.tick()
        self.end('rejected:rain')
        self.assertEqual(self.mc.persistent['last_run_by_event'], {})
        self.assertEqual(self.mc.persistent['run']['reason'], 'weather')
        self.clock[0] += dt.timedelta(minutes=30)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])          # still waiting
        self.clock[0] += dt.timedelta(hours=4)
        self.tick()
        self.assertEqual(self.starts(), ['Mow', 'Mow'])   # fresh, never a resume
        self.assertEqual(self.resumes(), [])

    def test_catch_up_within_limit_then_missed(self):
        self.mc.smach_status = 'Stopped'
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.assertEqual(self.mc._code, 'blocked_stopped')
        self.at(2026, 6, 10, 11, 0, 0)
        self.mc.smach_status = 'Ready'
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])          # caught up

    def test_missed_is_reported_once(self):
        self.mc.smach_status = 'Stopped'
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.at(2026, 6, 10, 12, 30, 0)
        self.tick(3)
        self.mc.smach_status = 'Ready'
        self.tick(3)
        self.assertEqual(self.starts(), [])
        issues = [i for i in self.mc.persistent['issues'] if i['code'] == 'slot_missed']
        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0]['detail']['blocked_by'], 'blocked_stopped')

    def test_robot_off_for_days_reports_and_runs_today_only(self):
        self.mc.persistent['scan_from'] = '2026-06-06T00:00:00'
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])
        self.assertEqual(len([i for i in self.mc.persistent['issues']
                              if i['code'] == 'slot_missed']), 4)

    def test_no_start_with_too_little_window_left(self):
        self.cal(daily('20:40'))
        self.at(2026, 6, 10, 20, 40, 5)
        self.tick()
        self.assertEqual(self.starts(), [])

    def test_clock_going_back_does_not_repeat(self):
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.run_started()
        self.end('succeeded')
        self.at(2026, 6, 10, 7, 59, 0)
        self.tick()
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_blocked_by_mission_agent_manual_dock(self):
        self.at(2026, 6, 10, 8, 0, 5)
        mc = self.mc
        for attr, val, code in (('mission_busy', True, 'blocked_mission'),
                                ('dock_status', 2, 'blocked_dock'),
                                ('mux_active', 'ds4', 'blocked_manual'),
                                ('agent_mission_active', True, 'blocked_agent')):
            old = getattr(mc, attr)
            setattr(mc, attr, val)
            self.tick()
            self.assertEqual(mc._code, code)
            self.assertEqual((self.starts(), self.sleeps), ([], []))
            setattr(mc, attr, old)
        mc.agent_status = {'state': 'READY', 'run_id': 'x'}
        mc.agent_status_t = mc._t()
        self.tick()
        self.assertEqual(mc._code, 'blocked_agent')
        self.clock[0] += dt.timedelta(seconds=60)          # stale: no agent there
        self.tick()
        self.assertEqual(self.starts(), ['Mow'])

    def test_program_not_on_map_retries_and_reports(self):
        self.mc.program_list = []
        self.mc.reject_max = 2
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.assertEqual(self.mc.persistent['run']['reject'], 'not_on_map')
        self.assertEqual(self.mc.persistent['last_run_by_event'], {})
        self.clock[0] += dt.timedelta(minutes=6)
        self.tick()
        self.assertIsNone(self.mc.persistent['run'])
        self.assertEqual(self.mc.persistent['issues'][-1]['code'], 'run_rejected')

    def test_not_picked_up_is_not_lost_silently(self):
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.mc._mowing_grace_until = 0
        self.mc.mission_seen = False
        self.clock[0] += dt.timedelta(seconds=70)
        self.tick()
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)
        self.assertEqual(self.mc.persistent['run']['reject'], 'not_started')

    def test_monthly_event_is_seen_beyond_a_week(self):
        ev = dict(daily('08:00'), recurrence='monthly', day_of_month=9)
        nd, _ = S.get_next_event([ev], D(2026, 6, 10, 9, 0, 0))
        self.assertEqual(nd, D(2026, 7, 9, 8, 0))


class Dst(unittest.TestCase):
    def test_real_seconds_over_both_changes(self):
        self.assertEqual(S.real_seconds(D(2027, 3, 27, 21, 30), D(2027, 3, 28, 7, 0)),
                         8.5 * 3600)
        self.assertEqual(S.real_seconds(D(2026, 10, 24, 21, 0), D(2026, 10, 25, 7, 0)),
                         11 * 3600)

    def test_slot_stays_on_the_wall_clock(self):
        slots = S.slots_between([daily('07:05')], D(2027, 3, 27, 21, 0), D(2027, 3, 28, 12, 0))
        self.assertEqual([s[0] for s in slots], [D(2027, 3, 28, 7, 5)])

    def test_slot_in_the_repeated_hour_runs_once(self):
        # 02:30 twice on 25. 10. 2026: slots are per date.
        slots = S.slots_between([daily('02:30')], D(2026, 10, 25, 0, 0), D(2026, 10, 25, 5, 0))
        self.assertEqual(len(slots), 1)


class Runs(Base):
    def setUp(self):
        Base.setUp(self)
        self.cal(daily('08:00'))
        self.on()
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.run_started()

    def again(self, minutes=1):
        self.clock[0] += dt.timedelta(minutes=minutes)
        self.tick()

    def test_battery_waits_for_the_target_then_resumes(self):
        mc = self.mc
        mc.program_list[0].last_result = 'failed: on_path-Z-1-200'
        mc.battery_capacity = 25
        self.end('interrupted:battery')
        self.assertEqual(self.sleeps, [(None, True)])
        mc.battery_capacity = 60
        self.again()
        self.assertEqual(mc._code, 'waiting_battery')
        self.assertEqual(self.resumes(), [])
        mc.battery_capacity = 80
        self.again()
        self.assertEqual(self.resumes(), ['Mow'])

    def test_charged_counts_below_the_target(self):
        self.mc.battery_capacity = 25
        self.end('interrupted:battery')
        self.mc.battery_capacity = 70
        self.mc.charger_status = 'CHARGED'
        self.again()
        self.assertEqual(self.resumes(), ['Mow'])

    def test_refused_charge_sleep_is_reported_once(self):
        mc = self.mc
        mc._execute_sleep = lambda duration_ms=None, charged=False: False
        mc.sleep_block = 'no_charging'
        mc.battery_capacity = 25
        self.end('interrupted:battery')
        self.again()
        self.again()
        self.assertEqual(len([i for i in mc.persistent['issues']
                              if i['code'] == 'not_charging']), 1)

    def _interrupt(self, reason, checkpoint):
        mc = self.mc
        mc.program_list[0].last_result = checkpoint
        self.end(reason)
        if mc.persistent['run'] is None:
            return False
        mc.persistent['run']['wait_until'] = 0
        mc.battery_capacity = 95
        self.again()
        self.run_started()
        return True

    def test_no_progress_cap(self):
        self.mc.no_progress_max = 3
        self.assertTrue(self._interrupt('interrupted:temp', 'failed: on_path-Z-1-0'))
        self.assertTrue(self._interrupt('interrupted:temp', 'failed: on_path-Z-1-0'))
        self.assertTrue(self._interrupt('interrupted:temp', 'failed: on_path-Z-1-0'))
        self.assertFalse(self._interrupt('interrupted:temp', 'failed: on_path-Z-1-0'))
        self.assertEqual(self.mc.persistent['issues'][-1]['code'], 'run_no_progress')

    def test_progress_resets_the_cap_but_not_the_total(self):
        self.mc.resume_max = 4
        for i in range(3):
            self.assertTrue(self._interrupt('interrupted:battery', 'failed: on_path-Z-%d-0' % i))
        self.assertFalse(self._interrupt('interrupted:battery', 'failed: on_path-Z-9-0'))
        self.assertEqual(self.mc.persistent['issues'][-1]['code'], 'run_too_many_resumes')

    def test_critical_cap_and_gps_stays_awake(self):
        self.mc.critical_max = 3
        self.assertTrue(self._interrupt('critical:gps_not_ready', 'failed: on_init'))
        self.assertEqual(self.sleeps, [])
        self.assertTrue(self._interrupt('critical:navigation_aborted', 'failed: on_path-Z-1-0'))
        self.assertFalse(self._interrupt('critical:navigation_aborted', 'failed: on_path-Z-2-0'))
        self.assertEqual(self.mc.persistent['issues'][-1]['code'], 'run_failed')

    def test_user_stop_ends_the_run(self):
        self.mc.smach_status = 'Stopped'
        self.mc.mission_busy = False
        self.tick()
        self.assertEqual(self.mc._code, 'blocked_stopped')
        self.end('stopped:user_stop')
        self.tick(2)
        self.assertEqual(self.resumes(), [])
        self.assertIsNone(self.mc.persistent['run'])

    def test_terminal_off_dock_stays_for_a_person(self):
        mc = self.mc
        mc.is_in_dock = False
        mc.smach_status = 'TERMINAL ERROR'
        mc.stop_reason = 'terminal:docking_failed'
        mc.terminal_short_wait_s = 0
        self.tick(3)
        self.clock[0] += dt.timedelta(hours=2)
        self.tick(3)
        self.assertEqual(published('/mower_smach/reset'), [])
        self.assertEqual(self.sleeps, [])
        self.assertEqual(mc._code, 'blocked_terminal')

    def test_lost_mission_goes_on_in_the_dock(self):
        mc = self.mc
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.stop_reason = ''
        mc.mission_lost_grace_s = 5
        self.tick()
        self.clock[0] += dt.timedelta(seconds=10)
        self.tick()
        self.assertEqual(mc.persistent['run']['reason'], 'lost')
        self.again()
        self.assertEqual(self.resumes(), ['Mow'])

    def test_window_end_finishes_the_path_then_caps(self):
        mc = self.mc
        self.at(2026, 6, 10, 21, 0, 30)
        self.tick()
        self.assertEqual(published('/mower_smach/finish_after_path'), [True])
        self.assertEqual(published('/mower_smach/return_home'), [])
        self.at(2026, 6, 10, 21, 31, 0)
        self.tick()
        self.assertEqual(published('/mower_smach/return_home'), ['window'])
        self.end('interrupted:window')
        self.assertEqual(mc.persistent['run']['reason'], 'window')
        self.tick()
        self.assertEqual(self.resumes(), [])
        self.assertEqual(len(self.sleeps), 1)              # sleeps to the morning
        self.at(2026, 6, 11, 7, 0, 30)
        self.tick()
        self.assertEqual(self.resumes(), ['Mow'])

    def test_new_event_waits_for_the_unfinished_run(self):
        mc = self.mc
        mc.calendar = [daily('08:00'), daily('12:00', 'ev2', 'Mow')]
        mc.battery_capacity = 95
        self.end('interrupted:temp')
        self.at(2026, 6, 10, 12, 0, 5)
        mc.persistent['run']['wait_until'] = 0
        self.tick()
        self.assertEqual(self.resumes(), ['Mow'])          # first the unfinished one
        self.run_started()
        self.end('succeeded')
        self.tick()
        self.assertEqual(self.starts(), ['Mow', 'Mow'])    # then the new event


class Restart(Base):
    def test_restart_attaches_to_a_running_mission(self):
        self.cal(daily('08:00'))
        self.on()
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.run_started()
        mc = self.mc = self.make()                         # node restarted
        mc.calendar = [daily('08:00')]
        mc.state = mc.STATE_IDLE
        mc.mission_busy = True
        mc.smach_status = 'ZoneA: exe path 1/3'
        self.tick(2)
        self.assertEqual(mc.state, mc.STATE_MOWING)
        self.end('succeeded')
        self.tick(3)
        self.assertEqual((self.starts(), self.resumes()), (['Mow'], []))

    def test_restart_after_the_mission_ended(self):
        self.cal(daily('08:00'))
        self.on()
        self.at(2026, 6, 10, 8, 0, 5)
        self.tick()
        self.run_started()
        mc = self.mc = self.make()
        mc.calendar = [daily('08:00')]
        mc.state = mc.STATE_IDLE
        mc.stop_reason = 'succeeded'
        mc.mission_started_name = 'Mow'
        self.tick(3)
        self.assertIsNone(mc.persistent['run'])
        self.assertEqual((self.starts(), self.resumes()), (['Mow'], []))

    def test_broken_state_file_uses_the_previous_generation_or_off(self):
        self.on()
        self.mc._save_state()
        path = self.mc.state_file
        with open(path, 'w') as f:
            f.write('{"automation_en')
        self.assertTrue(self.make().automation_on)          # .prev
        for p in (path, path + '.prev'):
            with open(p, 'w') as f:
                f.write('garbage')
        self.assertFalse(self.make().automation_on)

    def test_a_failed_write_keeps_the_good_file(self):
        self.on()
        mc = self.mc
        path = mc.state_file
        with open(path) as f:
            before = f.read()
        mc.persistent['bad'] = object()                     # not JSON
        mc._save_state()
        del mc.persistent['bad']
        with open(path) as f:
            self.assertEqual(f.read(), before)


class MapGate(Base):
    def test_silence_does_not_hold_the_start_for_ever(self):
        """vitulus-field#46: the planner died while it was loading a map."""
        mc = self.mc
        mc._mission_start_deadline = 0.0
        mc.start_state = {'state': 'switching', 'program': 'Mow', 'site': 'BACK'}
        mc.start_state_t = mc._t()
        self.assertTrue(mc._map_gate_says('Mow'))
        self.assertGreater(mc._mission_start_deadline, mc._t())
        self.clock[0] += dt.timedelta(seconds=mc.MAP_GATE_MAX_S + 1)
        self.assertFalse(mc._map_gate_says('Mow'))


if __name__ == '__main__':
    try:
        unittest.main(verbosity=1)
    finally:
        shutil.rmtree(_HOME, ignore_errors=True)
