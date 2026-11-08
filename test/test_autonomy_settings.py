#!/usr/bin/env python3
"""Autonomy page, backend: settings store, live apply, checks, sleep gates.

No ROS master: rospy is stubbed INCLUDING get_param/set_param (a real set_param
would reach whatever ROS_MASTER_URI says), the clock is the test's.

Run:  python3 test/test_autonomy_settings.py
"""
import datetime as dt
import glob
import importlib.machinery
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

import yaml

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HOME = tempfile.mkdtemp(prefix='autonomy_home_')
os.environ['HOME'] = _HOME
os.environ['TZ'] = 'Europe/Prague'
time.tzset()


def _navi():
    if os.environ.get('SIM_NAVI'):
        return os.environ['SIM_NAVI']
    parts = _PKG.split(os.sep)
    i = parts.index('vitulus_planner')
    return os.sep.join(parts[:i] + ['vitulus_navi'] + parts[i + 1:])


NAVI = _navi()
sys.path.insert(0, os.path.join(NAVI, 'src'))
sys.path.insert(0, os.path.join(_PKG, 'src'))

import rospy                                 # noqa: E402
from std_msgs.msg import Bool, String        # noqa: E402

EVENTS = []
PARAMS = {}          # the parameter server
YAML = {}            # ~param of master_controller


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        EVENTS.append((self.name, msg))


def _get_param(name, default=None):
    if name.startswith('~'):
        return YAML.get(name[1:], default)
    if name == '/node_mower_smach':
        return dict(PARAMS)
    return default


rospy.init_node = lambda *a, **kw: None
rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **kw: None
rospy.Service = lambda *a, **kw: None
rospy.get_param = _get_param
rospy.set_param = lambda name, value: PARAMS.__setitem__(name.rsplit('/', 1)[1], value)
rospy.sleep = lambda *a, **kw: None
for _l in ('loginfo', 'logwarn', 'logerr'):
    setattr(rospy, _l, lambda *a, **kw: None)
_loader = importlib.machinery.SourceFileLoader(
    'master_under_test3', os.path.join(_PKG, 'nodes', 'master_controller'))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
M = importlib.util.module_from_spec(_spec)
_loader.exec_module(M)
A = M.autonomy
SM = M.sleep_manager
D = dt.datetime


def last(topic):
    msgs = [m.data for t, m in EVENTS if t == topic]
    return msgs[-1] if msgs else None


def lastj(topic):
    return json.loads(last(topic))


def daily(t, eid='ev1', prog='Mow'):
    return {'id': eid, 'program_name': prog, 'enabled': True, 'recurrence': 'daily',
            'start_time': t}


def alert(age_s=0.0, now=None, **fields):
    m = types.SimpleNamespace(
        rain_now=0, status_now='OK', status_past60m='OK', status_past50m='OK',
        status_past40m='OK', status_past30m='OK', status_past20m='OK',
        status_past10m='OK', status_nowcast10m='OK', status_nowcast20m='OK',
        status_nowcast30m='OK')
    m.time = types.SimpleNamespace(to_sec=lambda: now - age_s)
    for k, v in fields.items():
        setattr(m, k, v)
    return m


class Base(unittest.TestCase):
    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        PARAMS.clear()
        YAML.clear()
        self.clock = [D(2026, 6, 10, 12, 0, 0)]
        self.mc = self.make()

    def make(self):
        mc = M.MasterController()
        mc._now = lambda: self.clock[0]
        mc._t = lambda: time.mktime(self.clock[0].timetuple())
        mc.node_start_time = mc._t() - 1000.0
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
        mc.boot_id_file = os.path.join(_HOME, 'boot_id')
        with open(mc.boot_id_file, 'w') as f:
            f.write('boot-1')
        self.starts = []
        mc.sleep_seq.start = lambda mode, ms, cfg=None: self.starts.append((mode, ms))
        self.logins = []
        mc._logins = lambda: list(self.logins)
        mc._wall = lambda text: None
        return mc

    def set(self, by='owner', reset=None, **values):
        msg = {'req': 'r1', 'by': by, 'values': values}
        if reset:
            msg['reset'] = reset
        self.mc.set_settings_callback(String(json.dumps(msg)))
        return lastj('/scheduler/settings_result')

    def on(self):
        self.mc.set_automation(True)
        self.mc.state = self.mc.STATE_IDLE

    def report(self):
        self.mc._publish_report(force=True)
        return lastj('/scheduler/report')

    def journal(self):
        self.report()
        return lastj('/scheduler/journal')['entries']

    def timeline(self):
        self.report()
        return lastj('/scheduler/timeline')

    def check(self, key):
        return next(c for c in self.report()['checks'] if c['key'] == key)

    def scheck(self, key):
        return next(c for c in self.report()['sleep']['checks'] if c['key'] == key)


class Defaults(Base):
    """Default behaviour = today: the yaml the robot runs is what the settings say."""

    def setUp(self):
        with open(os.path.join(_PKG, 'config', 'master_controller.yaml')) as f:
            self.yaml = yaml.safe_load(f)
        with open(os.path.join(NAVI, 'config', 'mower_smach.yaml')) as f:
            self.smach = yaml.safe_load(f)
        super().setUp()
        YAML.update(self.yaml)
        self.mc = self.make()

    def test_equal_to_todays_yaml(self):
        v = self.mc.settings
        y = self.yaml
        today = {
            'window_enabled': True, 'window_start': '07:00', 'window_end': '21:00',
            'window_min_left_min': 30.0, 'catchup_max_h': 4.0, 'skip_next': None,
            'no_mow_until': None, 'rain_mode': 'always', 'rain_after_min': 60,
            'rain_forecast_min': 30, 'rain_retry_h': 2.0, 'rain_return_home': True,
            'rain_stale_ok': True, 'battery_start_pct': 40, 'battery_return_pct': 25,
            'battery_resume_pct': 80, 'charge_sleep_max_h': 6.0, 'sleep_enabled': True,
            'sleep_ignore_ssh': False, 'sleep_ignore_ui': False,
            'idle_sleep_threshold_min': 20.0, 'wake_before_min': 12.0,
            'sleep_after_run': True, 'stay_awake_until': None, 'temp_wait_h': 1.0,
            'temp_spike_wait_min': 5.0, 'error_retry_max': 3, 'resume_max': 12,
            'run_max_age_h': 48.0, 'off_dock_wait_min': 20.0, 'auto_hw_reset': False,
            # calendar or agent (docs/AUTONOMY.md section 6): calendar unchanged
            'agent_fallback_calendar': True, 'limits_in_calendar': False,
            'limit_rain_h': 3.0, 'limit_temp_min_c': 5.0, 'limit_zone_gap_days': 2.0,
            'limit_day_start': '08:00', 'limit_day_end': '20:00'}
        self.assertEqual(v, today)
        self.assertEqual(set(v), set(A.BY_KEY))
        self.assertEqual(self.mc.wake_lead_s,
                         y['wake_before_program_s'] + y['boot_time_buffer_s'])

    def test_attributes_equal_the_yaml_after_apply(self):
        mc, y = self.mc, self.yaml
        mc._apply_settings(mc.settings)
        self.assertEqual(mc.idle_sleep_threshold_s, y['idle_sleep_threshold_s'])
        self.assertEqual(mc.off_dock_wait_s, y['off_dock_wait_s'])
        self.assertEqual(mc.catchup_max_s, y['catchup_max_h'] * 3600)
        self.assertEqual(mc.weather_sleep_duration_s, y['weather_sleep_duration_h'] * 3600)
        self.assertEqual(mc.battery_resume_min, y['battery_recovery_min_capacity'])
        self.assertEqual(mc.critical_max, y['critical_retry_max'])
        self.assertEqual(mc.sleep_check_logins, y['sleep_check_logins'])
        self.assertEqual(mc.sleep_check_ui, y['sleep_check_ui'])

    def test_mission_yaml_says_the_same(self):
        pushed = {p: self.mc.settings[k] for k, p in self.mc.MISSION_PARAMS.items()}
        self.assertEqual(pushed, {p: self.smach[p] for p in pushed})
        self.assertNotIn('weather_rain_bypass', self.smach)

    def test_nothing_changes_with_no_file(self):
        self.assertFalse(os.path.exists(os.path.join(_HOME, '.vitulus', 'autonomy.json')))


class Store(Base):
    def path(self):
        return os.path.join(_HOME, '.vitulus', 'autonomy.json')

    def test_survives_restart(self):
        self.assertTrue(self.set(temp_wait_h=2.0)['ok'])
        mc = self.make()
        self.assertEqual(mc.settings['temp_wait_h'], 2.0)
        self.assertEqual(mc.temp_sleep_duration_s, 7200.0)

    def test_published_with_schema(self):
        self.set(by='owner', resume_max=5)
        doc = lastj('/scheduler/settings')
        self.assertEqual(doc['values']['resume_max'], 5)
        self.assertEqual(doc['defaults']['resume_max'], 12)
        self.assertEqual(doc['by'], 'owner')
        self.assertEqual(doc['schema']['battery_return_pct']['mid_run'], 'next_run')
        self.assertTrue(doc['schema']['tilt_limit_deg']['locked'])
        self.assertEqual(doc['schema']['tilt_limit_deg']['locked_why'], 'tilt')
        self.assertIn('tilt_limit_deg', doc['values'])
        self.assertTrue(all(e['source'] for e in doc['schema'].values() if e['locked']))
        self.assertNotIn('battery_critical_pct', doc['schema'])
        self.assertFalse(any(s.get('needs_restart') for s in doc['schema'].values()))

    def test_bad_file_kept_aside(self):
        with open(self.path(), 'w') as f:
            f.write('{broken')
        mc = self.make()
        self.assertEqual(mc.settings, mc.settings_defaults)
        self.assertEqual(len(glob.glob(self.path() + '.bad-*')), 1)
        self.assertEqual(mc.persistent['issues'][-1]['code'], 'settings_bad')

    def test_bad_key_dropped_rest_kept(self):
        with open(self.path(), 'w') as f:
            json.dump({'version': 1, 'values': {'temp_wait_h': 2.0, 'resume_max': 'x'}}, f)
        mc = self.make()
        self.assertEqual(mc.settings['temp_wait_h'], 2.0)
        self.assertEqual(mc.settings['resume_max'], 12)
        self.assertEqual(len(glob.glob(self.path() + '.bad-*')), 1)

    def test_stored_values_cannot_undercut(self):
        with open(self.path(), 'w') as f:
            json.dump({'version': 1, 'values': {'battery_start_pct': 30,
                                                'battery_return_pct': 40}}, f)
        mc = self.make()
        self.assertEqual(mc.settings['battery_start_pct'], 40)
        self.assertEqual(mc.settings['battery_return_pct'], 25)

    def test_window_of_the_state_file_moves_in(self):
        self.mc.persistent['allowed_window'] = {'enabled': False, 'start': '08:00',
                                                'end': '20:00'}
        self.mc._save_state()
        mc = self.make()
        self.assertFalse(mc.settings['window_enabled'])
        self.assertEqual(mc.settings['window_start'], '08:00')
        self.assertTrue(os.path.exists(self.path()))
        mc = self.make()
        self.assertFalse(mc.window_enabled)

    def test_set_window_topic_still_works_and_is_one_value(self):
        self.mc.set_window_callback(String(json.dumps({'start': '08:30'})))
        self.assertEqual(self.mc.settings['window_start'], '08:30')
        self.assertEqual(self.mc.persistent['allowed_window']['start'], '08:30')
        self.assertTrue(self.set(window_end='19:00')['ok'])
        self.assertEqual(self.mc.window_end, (19, 0))
        self.assertEqual(self.mc.persistent['allowed_window']['end'], '19:00')


class Validation(Base):
    def test_all_or_nothing(self):
        r = self.set(temp_wait_h=2.0, resume_max=0)
        self.assertFalse(r['ok'])
        self.assertEqual(r['rejected'], {'resume_max': 'range'})
        self.assertEqual(self.mc.settings['temp_wait_h'], 1.0)

    def test_locked_and_unknown(self):
        r = self.set(tilt_limit_deg=45, nonsense=1)
        self.assertEqual(r['rejected'], {'tilt_limit_deg': 'locked', 'nonsense': 'unknown'})

    def test_battery_rules(self):
        self.assertEqual(self.set(battery_start_pct=19)['rejected'],
                         {'battery_start_pct': 'range'})
        self.assertEqual(self.set(battery_return_pct=45)['rejected'],
                         {'battery_start_pct': 'battery_start_under_return'})
        self.assertEqual(self.set(battery_resume_pct=30)['rejected'],
                         {'battery_resume_pct': 'battery_resume_under_start'})
        self.assertTrue(self.set(battery_start_pct=50, battery_return_pct=45)['ok'])
        self.assertEqual(self.set(battery_return_pct=14)['rejected'],
                         {'battery_return_pct': 'range'})

    def test_reset_goes_through_the_same_rules(self):
        self.assertTrue(self.set(battery_start_pct=25, battery_return_pct=20,
                                 battery_resume_pct=25)['ok'])
        self.assertTrue(self.set(reset=['battery_start_pct', 'battery_return_pct',
                                        'battery_resume_pct'])['ok'])
        self.assertEqual(self.mc.settings['battery_start_pct'], 40)
        self.assertNotIn('battery_start_pct', self.mc.settings_doc['values'])

    def test_types(self):
        self.assertEqual(self.set(rain_mode='sometimes')['rejected'], {'rain_mode': 'option'})
        self.assertEqual(self.set(rain_forecast_min=15)['rejected'], {'rain_forecast_min': 'range'})
        self.assertEqual(self.set(sleep_enabled=1)['rejected'], {'sleep_enabled': 'type'})
        self.assertEqual(self.set(resume_max=True)['rejected'], {'resume_max': 'type'})
        self.assertEqual(self.set(window_end='07:00')['rejected'], {'window_end': 'window_equal'})
        self.assertEqual(self.set(stay_awake_until='2026-06-10T11:00')['rejected'],
                         {'stay_awake_until': 'past'})
        self.assertEqual(self.set(stay_awake_until='2026-07-10T11:00')['rejected'],
                         {'stay_awake_until': 'too_far'})

    def test_bad_json(self):
        self.mc.set_settings_callback(String('[1'))
        self.assertFalse(lastj('/scheduler/settings_result')['ok'])
        self.mc.set_settings_callback(String('[1]'))
        self.assertFalse(lastj('/scheduler/settings_result')['ok'])


class LiveApply(Base):
    def test_each_key_reaches_what_is_read(self):
        mc = self.mc
        self.assertTrue(self.set(
            catchup_max_h=1.0, rain_retry_h=0.5, battery_start_pct=50,
            battery_resume_pct=90, charge_sleep_max_h=3.0, sleep_ignore_ssh=True,
            sleep_ignore_ui=True, idle_sleep_threshold_min=30.0, wake_before_min=5.0,
            temp_wait_h=2.0, temp_spike_wait_min=10.0, error_retry_max=5, resume_max=7,
            run_max_age_h=10.0, off_dock_wait_min=5.0, window_min_left_min=10.0,
            sleep_enabled=False, sleep_after_run=False)['ok'])
        self.assertEqual(
            (mc.catchup_max_s, mc.weather_sleep_duration_s, mc.battery_resume_min,
             mc.battery_resume_low, mc.sleep_cfg.charge_max_h, mc.sleep_check_logins,
             mc.sleep_check_ui, mc.idle_sleep_threshold_s, mc.wake_lead_s,
             mc.temp_sleep_duration_s, mc.temp_spike_wait_s, mc.critical_max, mc.resume_max,
             mc.run_max_age_s, mc.off_dock_wait_s, mc.start_min_left_s, mc.sleep_enabled,
             mc.sleep_after_run),
            (3600.0, 1800.0, 50, 90, 3.0, False, False, 1800.0, 300.0, 7200.0, 600.0,
             5, 7, 36000.0, 300.0, 600.0, False, False))

    def test_change_wakes_the_throttled_idle(self):
        self.mc._last_idle_eval = 1e12
        self.set(temp_wait_h=2.0)
        self.assertEqual(self.mc._last_idle_eval, 0.0)

    def test_window_change_while_mowing_is_noticed(self):
        self.mc.state = self.mc.STATE_MOWING
        self.set(window_end='13:00')
        self.assertTrue(self.mc._window_moved)

    def test_change_during_sleep_sequence_replans(self):
        self.mc.state = self.mc.STATE_PREPARE_SLEEP
        self.set(wake_before_min=5.0)
        self.assertTrue(self.mc._sleep_replan)


class Push(Base):
    def test_all_pushed(self):
        self.mc._push_params()
        self.assertEqual(PARAMS['weather_rain_mode'], 'always')
        self.assertEqual(PARAMS['battery_min_start_pct'], 40)
        self.assertEqual(PARAMS['battery_return_pct'], 25)

    def test_a_reload_is_put_right(self):
        self.set(rain_mode='calendar')
        self.mc._push_params()
        PARAMS['weather_rain_mode'] = 'always'      # roslaunch loaded the yaml
        self.mc._push_params()
        self.assertEqual(PARAMS['weather_rain_mode'], 'calendar')

    def test_return_level_waits_for_the_end_of_a_mission(self):
        mc = self.mc
        mc._push_params()
        mc.mission_busy = True
        self.set(battery_return_pct=30, rain_mode='ignore', battery_start_pct=50)
        mc._push_params()
        self.assertEqual(PARAMS['battery_return_pct'], 25)
        self.assertEqual(PARAMS['weather_rain_mode'], 'ignore')
        self.assertEqual(PARAMS['battery_min_start_pct'], 50)
        mc.mission_busy = False
        mc._push_params()
        self.assertEqual(PARAMS['battery_return_pct'], 30)

    def test_restart_into_a_running_mission_keeps_it(self):
        self.set(battery_return_pct=30)
        PARAMS['battery_return_pct'] = 25       # what the mission left with
        mc = self.make()
        mc.mission_busy = None                  # the node knows nothing yet
        mc._push_params()
        self.assertEqual(PARAMS['battery_return_pct'], 25)
        mc.mission_busy = True
        mc._push_params()
        self.assertEqual(PARAMS['battery_return_pct'], 25)

    def test_mission_values_are_read_back(self):
        PARAMS['weather_reject_statuses'] = ['RAIN', 'alert']
        PARAMS['weather_max_age_min'] = 10
        self.mc._push_params()
        self.assertEqual(self.mc._reject_statuses, ('RAIN', 'ALERT'))
        self.assertEqual(self.mc._weather_max_age_s, 600.0)


class Checks(Base):
    def test_keys(self):
        self.assertEqual([c['key'] for c in self.report()['checks']],
                         ['automation', 'hold', 'dock', 'mission', 'battery', 'rain',
                          'gnss', 'mower', 'temperature', 'window', 'programs', 'map'])

    def test_automation_hold_dock(self):
        self.assertEqual(self.check('automation')['state'], 'off')
        self.on()
        self.assertEqual(self.check('automation')['state'], 'ok')
        self.mc.is_inhibited = True
        self.assertEqual(self.check('hold')['code'], 'held')
        self.mc.is_in_dock = False
        self.assertEqual(self.check('dock')['state'], 'block')

    def test_mission_says_what_idle_says(self):
        mc = self.mc
        self.on()
        mc.calendar = [daily('23:00')]
        mc.dock_status = 1
        mc._step()
        self.assertEqual(mc._code, self.check('mission')['code'])
        self.assertEqual(self.check('mission')['code'], 'blocked_dock')
        mc.dock_status = 0
        mc.smach_status = 'Stopped'
        mc._step()
        self.assertEqual(mc._code, self.check('mission')['code'])
        mc.smach_status = 'Initiating'
        mc._step()
        self.assertEqual(mc._code, self.check('mission')['code'])
        self.assertEqual(self.check('mission')['vars']['smach'], 'Initiating')

    def test_check_is_current_inside_the_idle_throttle(self):
        mc = self.mc
        self.on()
        mc.check_interval_s = 1e9
        mc._last_idle_eval = mc._t()
        mc.mission_busy = True
        self.assertEqual(self.check('mission')['code'], 'blocked_mission')
        mc.mission_busy = False
        self.assertEqual(self.check('mission')['code'], 'ready')

    def test_mux_hold_is_gnss(self):
        self.mc.mux_active = 'hold'
        self.assertEqual(self.check('gnss')['code'], 'mux_hold')
        self.assertEqual(self.check('mission')['code'], 'ready')
        self.assertEqual(self.mc._busy_verdict(), ('blocked_mux_hold', {}))

    def test_battery_same_limit_as_the_start(self):
        mc = self.mc
        mc.pm_status = object()
        mc.battery_capacity = 39
        c = self.check('battery')
        self.assertEqual((c['state'], c['vars']['need']), ('wait', 40))
        self.set(battery_start_pct=30)
        self.assertEqual(self.check('battery')['state'], 'ok')
        self.assertIsNone(mc._battery_short(None))

    def test_window(self):
        self.set(window_start='13:00')
        self.assertEqual(self.check('window')['code'], 'outside_window')
        self.set(window_start='11:00', window_end='12:10')
        self.assertEqual(self.check('window')['code'], 'too_little_left')
        self.assertFalse(self.mc._start_allowed(self.clock[0]))
        self.set(window_enabled=False)
        self.assertEqual(self.check('window')['state'], 'off')

    def test_programs(self):
        self.assertEqual(self.check('programs')['code'], 'no_schedule')
        self.mc.calendar = [daily('23:00', prog='Gone')]
        self.mc.site_programs = {}
        self.assertEqual(self.check('programs')['code'], 'not_on_map')
        self.mc.calendar = [daily('23:00')]
        self.assertEqual(self.check('programs')['code'], 'ok')

    def test_waits_of_a_run(self):
        mc = self.mc
        for reason, key, code in (('weather', 'rain', 'waiting_weather'),
                                  ('temp', 'temperature', 'waiting_temp'),
                                  ('temp_spike', 'temperature', 'waiting_temp_spike'),
                                  ('gps', 'gnss', 'waiting_gps'),
                                  ('critical', 'mission', 'waiting_critical')):
            mc.persistent['run'] = {'program': 'Mow', 'reason': reason,
                                    'wait_until': mc._t() + 300}
            c = self.check(key)
            self.assertEqual((c['state'], c['code'], c['vars']['left_s']), ('wait', code, 300))

    def test_temp_spike_wait_is_reported_and_stays_awake(self):
        mc = self.mc
        self.on()
        mc.calendar = [daily('23:00')]
        run = mc._new_run(mc.calendar[0], self.clock[0])
        run['started'] = run['following'] = True
        mc.persistent['run'] = run
        slept = []
        mc._execute_sleep = lambda **k: slept.append(k) or True
        mc._mission_seen = True
        mc._handle_program_end('interrupted:temp_spike', True)
        self.assertEqual(slept, [])
        mc._step()
        self.assertEqual(mc._code, 'waiting_temp_spike')
        rep = self.report()
        self.assertEqual(rep['sentence']['code'], 'waiting_temp_spike')
        self.assertEqual(rep['sentence']['vars']['program'], 'Mow')
        self.assertEqual(rep['run']['reason'], 'temp_spike')

    def test_run_reject(self):
        mc = self.mc
        self.on()
        mc.calendar = [daily('23:00')]
        run = mc._new_run(mc.calendar[0], self.clock[0])
        mc.persistent['run'] = run
        mc._handle_program_end('rejected:recent_rain', True)
        self.assertEqual(self.report()['run']['reject'], 'recent_rain')
        mc._after_rejected(mc._run(), 'map:no_file')
        self.assertEqual(self.report()['run']['reject'], 'map:no_file')

    def test_manual_refusal_is_in_the_report(self):
        self.mc.stop_reason = 'rejected:rain'
        self.assertEqual(self.report()['reject'], 'rain')

    def test_rain_stale_rejection_waits_like_rain(self):
        mc = self.mc
        self.on()
        mc.calendar = [daily('23:00')]
        mc.persistent['run'] = mc._new_run(mc.calendar[0], self.clock[0])
        mc._handle_program_end('rejected:rain_stale', True)
        self.assertEqual(mc._run()['reason'], 'weather')


class Rain(Base):
    def feed(self, age_s=0.0, **fields):
        self.mc.weather_msg = alert(age_s, self.mc._t(), **fields)

    def test_no_data_is_unknown_and_no_refusal(self):
        self.assertEqual(self.check('rain')['state'], 'unknown')
        self.assertIsNone(self.report()['rain_would_refuse'])

    def test_same_codes_as_the_mission(self):
        self.feed()
        self.assertEqual(self.check('rain')['code'], 'dry')
        for fields, code in (({'rain_now': 1}, 'rain'),
                             ({'status_past40m': 'RAIN'}, 'recent_rain'),
                             ({'status_nowcast20m': 'RAIN'}, 'forecast')):
            self.feed(**fields)
            self.assertEqual(self.check('rain')['code'], code)
            self.assertEqual(self.report()['rain_would_refuse'], code)

    def test_settings_move_the_rule(self):
        self.feed(status_past40m='RAIN')
        self.set(rain_after_min=30)
        self.assertEqual(self.check('rain')['code'], 'dry')
        self.feed(status_nowcast30m='RAIN')
        self.set(rain_forecast_min=20)
        self.assertEqual(self.check('rain')['code'], 'dry')

    def test_modes(self):
        self.feed(rain_now=1)
        self.set(rain_mode='calendar')
        self.assertEqual(self.check('rain')['code'], 'rain')    # a calendar start obeys
        self.assertIsNone(self.report()['rain_would_refuse'])   # a Run does not
        self.set(rain_mode='ignore')
        self.assertEqual(self.check('rain')['code'], 'ignored_mode')
        self.assertIsNone(self.report()['rain_would_refuse'])

    def test_stale(self):
        self.feed(age_s=3600.0, rain_now=1)
        self.assertEqual(self.check('rain')['state'], 'unknown')
        self.assertIsNone(self.report()['rain_would_refuse'])
        self.set(rain_stale_ok=False)
        self.assertEqual(self.check('rain')['state'], 'block')
        self.assertEqual(self.report()['rain_would_refuse'], 'rain_stale')

    def test_once_until_the_mission_ends(self):
        mc = self.mc
        self.feed(rain_now=1)
        mc.ignore_rain_callback(Bool(True))
        rep = self.report()
        self.assertTrue(rep['ignore_rain_once'])
        self.assertIsNone(rep['rain_would_refuse'])
        mc.mission_busy = True
        mc._tick_ignore_rain()
        self.clock[0] += dt.timedelta(hours=1)
        mc._tick_ignore_rain()
        self.assertTrue(self.report()['ignore_rain_once'])
        mc.mission_busy = False
        mc._tick_ignore_rain()
        self.assertFalse(self.report()['ignore_rain_once'])

    def test_once_unused_runs_out(self):
        mc = self.mc
        mc.ignore_rain_callback(Bool(True))
        self.clock[0] += dt.timedelta(seconds=301)
        mc._tick_ignore_rain()
        self.assertFalse(self.report()['ignore_rain_once'])

    def test_once_disarmed(self):
        self.mc.ignore_rain_callback(Bool(True))
        self.mc.ignore_rain_callback(Bool(False))
        self.assertFalse(self.report()['ignore_rain_once'])


class SleepGates(Base):
    def sleep(self, **kw):
        kw.setdefault('duration_ms', 3600000)
        return self.mc._execute_sleep(**kw)

    def setUp(self):
        super().setUp()
        self.on()

    def test_list_and_gate_are_one(self):
        keys = [c['key'] for c in self.report()['sleep']['checks']]
        self.assertEqual(keys, ['enabled', 'hold', 'dock', 'mission', 'manual', 'agent',
                                'ssh', 'ui', 'boot', 'stay_awake', 'reason'])
        self.assertIsNone(self.mc._sleep_gate())

    def test_sleep_off(self):
        self.set(sleep_enabled=False)
        self.assertIs(self.sleep(), False)
        self.assertEqual(self.report()['sleep']['block'], 'sleep_off')
        self.assertEqual(self.scheck('enabled')['code'], 'sleep_off')
        self.set(sleep_enabled=True)
        self.assertIsNone(self.report()['sleep']['block'])

    def test_block_clears_when_the_cause_goes(self):
        mc = self.mc
        mc.is_in_dock = False
        self.assertIs(self.sleep(), False)
        self.assertEqual(self.report()['sleep']['block'], 'off_dock')
        mc.is_in_dock = True
        self.assertIsNone(self.report()['sleep']['block'])
        self.assertEqual(mc.sleep_block, 'off_dock')     # the old stale field

    def test_login_block_clears(self):
        self.logins = ['pts/1']
        self.assertIs(self.sleep(), False)
        self.assertEqual(self.mc._sleep_gate(), 'login:pts/1')
        self.logins = []
        self.mc._logins_cache = (0.0, [])
        self.assertIsNone(self.report()['sleep']['block'])

    def test_too_short_shows_then_goes(self):
        self.assertIs(self.sleep(duration_ms=1000), False)
        self.assertEqual(self.report()['sleep']['block'], 'too_short')
        self.mc.check_interval_s = 30
        self.clock[0] += dt.timedelta(minutes=5)
        self.assertIsNone(self.report()['sleep']['block'])

    def test_report_path_runs_no_process(self):
        mc = self.mc
        mc._logins = lambda: self.fail('who on a callback thread')
        mc._logins_cache = (0.0, ['pts/9'])
        mc._publish_report(force=True)
        self.assertEqual(lastj('/scheduler/report')['sleep']['block'], 'login:pts/9')

    def test_ssh_ignored_mission_still_gates(self):
        mc = self.mc
        self.set(sleep_ignore_ssh=True)
        self.logins = ['pts/1']
        mc.mission_busy = True
        self.assertEqual(mc._sleep_gate(), 'busy:mission')
        mc.mission_busy = False
        mc.mux_active = 'ds4'
        self.assertEqual(mc._sleep_gate(), 'busy:manual')
        self.assertEqual(self.scheck('manual')['state'], 'block')
        mc.mux_active = 'nav'
        mc.dock_status = 2
        self.assertEqual(mc._sleep_gate(), 'busy:dock')
        mc.dock_status = 0
        self.assertIsNone(mc._sleep_gate())
        mc._logins_cache = (0.0, ['pts/1'])
        self.assertEqual(self.scheck('ssh')['code'], 'ignored')

    def test_driver_who_let_go_counts_when_ignoring(self):
        mc = self.mc
        self.set(sleep_ignore_ui=True)
        mc.mux_callback(String('teleop'))
        mc.mux_callback(String('nav'))
        self.assertEqual(mc._sleep_gate(), 'busy:manual')
        self.clock[0] += dt.timedelta(seconds=121)
        self.assertIsNone(mc._sleep_gate())

    def test_default_has_no_quiet_time(self):
        mc = self.mc
        mc.mux_callback(String('teleop'))
        mc.mux_callback(String('nav'))
        self.assertIsNone(mc._sleep_gate())     # as today

    def test_ui_gate_default_and_ignored(self):
        mc = self.mc
        mc.ui_clients = 1
        self.assertEqual(mc._sleep_gate(), 'ui:1')
        self.set(sleep_ignore_ui=True)
        self.assertIsNone(mc._sleep_gate())
        self.assertEqual(self.scheck('ui')['code'], 'ignored')

    def test_stay_awake_survives_restart_and_runs_out(self):
        self.assertTrue(self.set(stay_awake_until='2026-06-10T13:00')['ok'])
        self.assertEqual(self.mc._sleep_gate(), 'stay_awake')
        mc = self.mc = self.make()
        self.on()
        self.assertEqual(mc._sleep_gate(), 'stay_awake')
        self.assertEqual(self.scheck('stay_awake')['vars']['until'], '2026-06-10T13:00')
        self.clock[0] = D(2026, 6, 10, 13, 0, 1)
        mc._step()
        self.assertIsNone(mc.settings['stay_awake_until'])
        self.assertIsNone(mc._sleep_gate())

    def test_clock_back_cannot_pin_stay_awake(self):
        self.set(stay_awake_until='2026-06-11T13:00')
        self.clock[0] = D(2026, 1, 1, 12, 0, 0)
        self.mc._step()
        self.assertIsNone(self.mc.settings['stay_awake_until'])

    def test_a_setting_cancels_a_sleep_on_its_way(self):
        mc = self.mc
        self.assertIs(self.sleep(), True)
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)
        self.set(sleep_enabled=False)
        self.assertEqual(mc._sleep_gate(), 'sleep_off')

    def test_sleep_after_run_off(self):
        mc = self.mc
        mc.calendar = [daily('23:00')]
        slept = []
        mc._sleep_for = lambda *a: slept.append(a)
        mc._step()
        self.assertEqual(len(slept), 1)         # no run yet: as ever
        del slept[:]
        self.set(sleep_after_run=False)
        mc._route_after_program(None, True)
        mc._step()
        self.assertEqual(slept, [])
        mc.persistent['after_run_boot'] = 'an older boot'
        mc._step()
        self.assertEqual(len(slept), 1)

    def test_sleep_after_run_default_sleeps(self):
        mc = self.mc
        mc.calendar = [daily('23:00')]
        slept = []
        mc._sleep_for = lambda *a: slept.append(a)
        mc._route_after_program(None, True)
        mc._step()
        self.assertEqual(len(slept), 1)


class HwReset(Base):
    """node_planner/hw_reset.py in the master, on the fake robot."""

    def setUp(self):
        super().setUp()
        self.on()
        mc = self.mc
        self.set(auto_hw_reset=True, sleep_ignore_ssh=True, sleep_ignore_ui=True)
        mc.persistent['run'] = dict(mc._new_run({'id': 'e', 'program_name': 'Mow'}, None),
                                    reason='mower_link', started=True, wait_until=mc._t() + 600)
        mc.mower_link = {'state': 'silent', 'silent_s': 500.0}
        mc.mower_link_t = mc._t()
        mc.pm_status = type('P', (), dict(supply_status='ONLINE', sleep_status=False,
                                          sleep_charged_status=False, sleep_wait_standby=180000,
                                          sleep_time_interval=900000, battery_capacity=90,
                                          charger_status='CHARGING'))()
        mc.pm_status_time = mc._t()
        mc.tty_active = lambda logins: []

    def cd(self):
        return lastj('/scheduler/sleep_countdown')

    def test_h4_off_by_default(self):
        self.assertIs(M.autonomy.defaults(lambda k, d=None: d)['auto_hw_reset'], False)

    def test_announced_with_its_own_notify_code(self):
        self.assertTrue(self.mc._tick_hw_reset())
        self.assertEqual(self.cd()['reason'], 'hw_reset')
        self.assertEqual(lastj('/scheduler/notify')['code'], 'mower_link')

    def test_m6_only_spent_wants_a_person(self):
        routes = self.mc.ISSUE_ROUTES
        self.assertEqual(routes['hw_reset_spent'][1], 'hw_reset_spent')
        for code in ('mower_link', 'hw_reset', 'hw_reset_person'):
            self.assertEqual(routes[code][1], 'mower_link')

    def test_h3_a_live_ignored_login_stops_the_cut(self):
        self.logins = ['pts/1']
        self.mc.tty_active = lambda logins: list(logins)
        self.assertFalse(self.mc._tick_hw_reset())
        self.assertFalse(self.cd()['active'])
        self.assertEqual(self.mc.persistent['issues'][-1]['code'], 'hw_reset_person')

    def test_h3_a_dead_ignored_login_does_not(self):
        self.logins = ['pts/0']
        self.assertTrue(self.mc._tick_hw_reset())

    def test_h3_somebody_logs_in_during_the_countdown(self):
        self.assertTrue(self.mc._tick_hw_reset())
        self.logins = ['pts/1']
        self.mc.tty_active = lambda logins: list(logins)
        self.mc._tick_sleep_requests()
        self.assertFalse(self.cd()['active'])
        self.assertEqual(self.starts, [])

    def test_low_off_s_never_under_w(self):
        with mock.patch.dict(YAML, {'hw_reset_off_s': 0.0}):
            mc = self.make()
        self.assertGreater(mc.hw_reset_ms, mc.hw_cfg.wait_before_standby_s * 1000)

    def _armed(self):
        mc = self.mc
        mc._sleep_go(M.sleep_manager.MODE_TIMED, mc.hw_reset_ms, 'hw_reset', {})
        return mc

    def test_low_a_second_go_counts_once(self):
        mc = self._armed()
        mc._sleep_go(M.sleep_manager.MODE_TIMED, mc.hw_reset_ms, 'hw_reset', {})
        self.assertEqual(len(mc.persistent['hw_resets']), 1)

    def test_low_a_cancel_in_the_sequence_does_not_count(self):
        mc = self._armed()
        mc._hw_reset_cancelled()
        self.assertEqual(mc.persistent.get('hw_resets'), [])
        self.assertEqual(mc._run()['hw_reset'], 'cancelled')


class Countdown(Base):
    def setUp(self):
        super().setUp()
        self.on()
        self.set(sleep_ignore_ssh=True)
        self.logins = ['pts/1']

    def cd(self):
        return lastj('/scheduler/sleep_countdown')

    def go(self):
        return self.mc._execute_sleep(duration_ms=3600000)

    def ticks(self, n):
        for _ in range(n):
            self.mc._tick_sleep_requests()

    def test_no_people_no_countdown(self):
        self.logins = []
        self.assertIs(self.go(), True)
        self.assertEqual(len(self.starts), 1)
        self.assertFalse(self.cd()['active'])

    def test_announced_then_sleeps(self):
        mc = self.mc
        self.assertIs(self.go(), False)
        self.assertEqual(self.starts, [])
        c = self.cd()
        self.assertEqual((c['active'], c['left_s'], c['who']['ssh']), (True, 60, ['pts/1']))
        self.ticks(30)
        self.assertEqual(self.cd()['left_s'], 30)
        self.assertEqual(self.report()['sleep']['block'], 'countdown')
        self.ticks(30)
        self.assertEqual(len(self.starts), 1)
        self.assertEqual(self.starts[0][1], 3600000 * 0.995 // 1 - 60000)
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)
        self.assertFalse(self.cd()['active'])

    def test_a_clock_jump_does_not_shorten_it(self):
        self.go()
        self.clock[0] += dt.timedelta(hours=2)
        self.ticks(1)
        self.assertEqual(self.starts, [])
        self.assertEqual(self.cd()['left_s'], 59)

    def test_person_cancels(self):
        mc = self.mc
        self.go()
        self.ticks(5)
        mc.sleep_cancel_callback(Bool(True))
        self.ticks(1)
        self.assertFalse(self.cd()['active'])
        self.ticks(70)
        self.assertEqual(self.starts, [])
        self.assertEqual(self.report()['sleep']['block'], 'countdown_cancelled')
        self.assertIs(self.go(), False)         # not again right away
        self.assertFalse(self.cd()['active'])

    def test_mission_start_cancels(self):
        mc = self.mc
        self.go()
        mc.mission_busy = True
        self.ticks(1)
        self.assertFalse(self.cd()['active'])
        self.assertEqual(mc.sleep_block, 'busy:mission')
        self.ticks(70)
        self.assertEqual(self.starts, [])

    def test_a_start_in_the_countdown_wins(self):
        mc = self.mc
        self.go()
        mc.state = mc.STATE_MOWING      # the mission has not said busy yet
        self.ticks(1)
        self.assertFalse(self.cd()['active'])
        self.ticks(70)
        self.assertEqual((self.starts, mc.state), ([], mc.STATE_MOWING))

    def test_one_countdown_only(self):
        self.go()
        self.ticks(10)
        self.assertIs(self.go(), False)
        self.assertEqual(self.cd()['left_s'], 50)

    def test_cancel_during_the_sequence(self):
        mc = self.mc
        self.logins = []
        self.go()
        seq = mc.sleep_seq
        seq.state = SM.PARAMS
        mc.pm_status = None
        mc._sleep_cancel_req = True
        mc._handle_prepare_sleep()
        self.assertEqual(seq.state, SM.IDLE)
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.assertEqual(mc._sleep_cooldown_reason, 'countdown_cancelled')


class SleepNow(Base):
    def now(self, **kw):
        self.mc.sleep_now_callback(String(json.dumps(kw)))
        self.mc._tick_sleep_requests()
        return lastj('/scheduler/command_result')

    def setUp(self):
        super().setUp()
        self.on()

    def test_sleeps_with_automation_off_and_under_hold(self):
        mc = self.mc
        mc.set_automation(False)
        mc.state = mc.STATE_DISABLED
        r = self.now(mode='timed', ms=3600000)
        self.assertEqual((r['ok'], r['code']), (True, 'sleeping'))
        self.assertIsNone(mc._sleep_gate())         # 'disabled' must not abort it
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)

    def test_under_hold(self):
        mc = self.mc
        mc.is_inhibited = True
        mc.state = mc.STATE_INHIBITED
        self.assertTrue(self.now(mode='timed', ms=3600000)['ok'])

    def test_gates_of_the_robot_hold(self):
        mc = self.mc
        mc.is_in_dock = False
        self.assertEqual(self.now(mode='timed', ms=3600000),
                         {'cmd': 'sleep_now', 'ok': False, 'code': 'off_dock'})
        mc.is_in_dock = True
        mc.mission_busy = True
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'busy:mission')
        mc.mission_busy = False
        mc.mux_active = 'teleop'
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'busy:manual')
        self.assertEqual(self.starts, [])

    def test_states_that_refuse(self):
        mc = self.mc
        mc.state = mc.STATE_MOWING
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'busy:MOWING')
        mc.state = mc.STATE_WAIT_OFF_DOCK
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'off_dock')

    def test_ssh_counts_unless_ignored(self):
        self.logins = ['pts/1']
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'login:pts/1')
        self.set(sleep_ignore_ssh=True)
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'countdown')

    def test_the_ui_that_pressed_it_does_not_stop_it(self):
        mc = self.mc
        mc.ui_clients = 2
        r = self.now(mode='timed', ms=3600000)
        self.assertEqual((r['ok'], r['code']), (True, 'countdown'))
        self.assertEqual(lastj('/scheduler/sleep_countdown')['who']['ui'], 2)
        for _ in range(60):
            mc._tick_sleep_requests()
        self.assertEqual(len(self.starts), 1)
        self.assertIsNone(mc._sleep_gate())     # UI still open: not cancelled
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)

    def test_ignores_a_stay_awake_and_the_cooldown(self):
        mc = self.mc
        self.set(stay_awake_until='2026-06-10T13:00')
        mc._sleep_hold_off('pm_lost')
        self.assertEqual(self.now(mode='timed', ms=3600000)['code'], 'sleeping')

    def test_still_wakes_for_the_next_program(self):
        self.mc.calendar = [daily('14:00')]
        self.now(mode='timed', ms=10 * 3600000)
        self.assertLessEqual(self.starts[0][1], (2 * 3600 - 720) * 1000)

    def test_too_short(self):
        self.assertEqual(self.now(mode='timed', ms=1000),
                         {'cmd': 'sleep_now', 'ok': False, 'code': 'too_short'})

    def test_charged(self):
        mc = self.mc
        mc.pm_status = types.SimpleNamespace(
            charger_status='CHARGING', supply_status='ONLINE', battery_capacity=50)
        self.assertEqual(self.now(mode='charged')['code'], 'sleeping')
        self.assertEqual(self.starts[0][0], SM.MODE_TIMED)

    def test_invalid(self):
        for bad in ({'mode': 'x'}, {'mode': 'timed'}, {'mode': 'timed', 'ms': -1},
                    {'mode': 'timed', 'ms': True}):
            self.assertEqual(self.now(**bad)['code'], 'invalid')
        self.mc.sleep_now_callback(String('[1]'))
        self.assertEqual(lastj('/scheduler/command_result')['code'], 'invalid')


class Skips(Base):
    def setUp(self):
        super().setUp()
        self.on()
        self.mc.calendar = [daily('14:00')]
        self.mc.window_enabled = False
        self.slept = []
        self.mc._sleep_for = lambda *a: self.slept.append(a)

    def selects(self):
        return [m.data for t, m in EVENTS if t == '/web_plan/program_select']

    def test_skip_pins_the_next_slot(self):
        self.assertTrue(self.set(skip_next=True)['ok'])
        self.assertEqual(self.mc.settings['skip_next'], '2026-06-10T14:00')
        self.assertEqual(self.report()['next']['at'], '2026-06-11T14:00')
        self.assertTrue(self.set(skip_next=True)['ok'])         # does not walk on
        self.assertEqual(self.mc.settings['skip_next'], '2026-06-10T14:00')
        self.assertTrue(self.set(skip_next=False)['ok'])
        self.assertIsNone(self.mc.settings['skip_next'])

    def test_nothing_to_skip(self):
        self.mc.calendar = []
        self.assertEqual(self.set(skip_next=True)['rejected'], {'skip_next': 'no_slot'})

    def test_skipped_slot_does_not_run_and_the_pin_ends(self):
        mc = self.mc
        self.set(skip_next=True)
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._step()
        self.assertEqual(self.selects(), [])
        self.assertIsNone(mc.settings['skip_next'])
        mc._step()
        self.assertEqual(self.selects(), [])
        self.assertEqual(mc.persistent['closed_slot_by_event']['ev1'], '2026-06-10')
        skipped = [j for j in self.journal() if j['kind'] == 'skipped']
        self.assertEqual(skipped[0]['code'], 'skip_next')

    def test_pin_survives_a_mission_over_its_slot(self):
        mc = self.mc
        self.set(skip_next=True)
        mc.state = mc.STATE_MOWING
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._expire_settings(self.clock[0])
        self.assertIsNotNone(mc.settings['skip_next'])

    def test_skip_survives_restart(self):
        self.set(skip_next=True)
        mc = self.make()
        self.assertEqual(mc.settings['skip_next'], '2026-06-10T14:00')

    def test_sleep_goes_past_the_skipped_slot(self):
        mc = self.mc
        self.set(skip_next=True)
        mc._step()
        self.assertEqual(len(self.slept), 1)
        self.assertGreater(self.slept[0][1], 20 * 3600)

    def test_default_next_is_as_ever(self):
        mc = self.mc
        self.assertEqual(mc._next_slot(self.clock[0]),
                         M.MowerScheduler.get_next_event(mc.calendar, self.clock[0]))

    def test_no_mow_today(self):
        mc = self.mc
        self.assertTrue(self.set(no_mow_until='2026-06-10')['ok'])
        self.assertEqual(self.report()['next']['at'], '2026-06-11T14:00')
        self.assertEqual(self.check('programs')['code'], 'no_mow')
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._step()
        self.assertEqual(self.selects(), [])
        self.clock[0] = D(2026, 6, 11, 0, 0, 5)
        mc._step()
        self.assertIsNone(mc.settings['no_mow_until'])
        self.clock[0] = D(2026, 6, 11, 14, 0, 30)
        mc.mission_busy = False
        mc._step()
        self.assertEqual(self.selects(), ['Mow'])

    def test_no_mow_survives_restart_and_holds_a_run_back(self):
        self.set(no_mow_until='2026-06-11')
        mc = self.mc = self.make()
        self.on()
        mc.calendar = [daily('14:00')]
        mc.window_enabled = False
        mc._sleep_for = lambda *a: None
        mc.persistent['run'] = mc._new_run(mc.calendar[0], self.clock[0])
        mc._step()
        self.assertEqual(mc._code, 'no_mow')
        self.assertEqual(self.selects(), [])

    def test_no_mow_past_is_refused(self):
        self.assertEqual(self.set(no_mow_until='2026-06-09')['rejected'],
                         {'no_mow_until': 'past'})

    def test_long_no_mow_finds_the_slot_after_it(self):
        self.set(no_mow_until='2026-08-01')
        self.assertEqual(self.mc._next_slot(self.clock[0])[0], D(2026, 8, 2, 14, 0))

    def test_run_next(self):
        mc = self.mc
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        self.assertEqual(lastj('/scheduler/command_result'),
                         {'cmd': 'run_next', 'ok': True, 'code': 'starting'})
        self.assertEqual(self.selects(), ['Mow'])
        self.assertEqual(mc._run()['slot'], '2026-06-10T14:00')
        mc.mission_started_name = 'Mow'
        mc._confirm_started('Mow')
        mc.persistent['run'] = None
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc.state = mc.STATE_IDLE
        del EVENTS[:]
        mc._step()
        self.assertEqual(self.selects(), [])    # its slot is used up

    def test_run_next_says_why_not(self):
        mc = self.mc
        mc.mission_busy = True
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        self.assertEqual(lastj('/scheduler/command_result')['code'], 'mission:blocked_mission')
        mc.mission_busy = False
        mc.is_inhibited = True
        mc.state = mc.STATE_INHIBITED
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        self.assertEqual(lastj('/scheduler/command_result')['code'], 'busy:INHIBITED')
        self.assertEqual(self.selects(), [])

    def test_run_next_with_automation_off(self):
        mc = self.mc
        mc.set_automation(False)
        mc.state = mc.STATE_DISABLED
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        self.assertFalse(lastj('/scheduler/command_result')['ok'])


class Timeline(Base):
    def setUp(self):
        super().setUp()
        self.on()
        self.mc.calendar = [daily('14:00')]

    def slots(self):
        return self.timeline()['starts']

    def test_seven_days(self):
        s = self.slots()
        self.assertEqual(len(s), 7)
        self.assertEqual(s[0], {'t': '2026-06-10T14:00', 'program': 'Mow', 'site': None,
                                'event_id': 'ev1', 'state': 'planned'})

    def test_skip_and_no_mow(self):
        self.set(skip_next=True)
        self.assertEqual((self.slots()[0]['state'], self.slots()[0]['reason']),
                         ('skip', 'skip_next'))
        self.set(skip_next=False, no_mow_until='2026-06-11')
        self.assertEqual([s['state'] for s in self.slots()][:3],
                         ['no_mow', 'no_mow', 'planned'])

    def test_automation_and_hold(self):
        self.mc.is_inhibited = True
        self.assertEqual(self.slots()[0]['reason'], 'held')
        self.assertEqual(self.timeline()['sleeps'], [])
        self.mc.is_inhibited = False
        self.mc.automation_on = False
        self.assertEqual(self.slots()[0]['reason'], 'automation_off')

    def test_outside_window(self):
        self.set(window_start='15:00', window_end='20:00')
        s = self.slots()[0]
        self.assertEqual((s['state'], s['at']), ('planned', '2026-06-10T15:00'))
        self.set(window_start='19:00', catchup_max_h=1.0)
        s = self.slots()[0]
        self.assertEqual((s['state'], s['reason']), ('outside', 'outside_window'))

    def test_sleeps(self):
        s = self.timeline()['sleeps']
        self.assertEqual(s[0], {'from': '2026-06-10T12:00', 'to': '2026-06-10T13:48',
                                'reason': 'next_slot'})
        self.assertEqual(s[1]['from'], '2026-06-10T14:00')
        self.set(sleep_enabled=False)
        self.assertEqual(self.timeline()['sleeps'], [])

    def test_sleep_after_run_off_leaves_only_the_first(self):
        self.set(sleep_after_run=False)
        self.assertEqual(len(self.timeline()['sleeps']), 1)


class Journal(Base):
    def setUp(self):
        super().setUp()
        self.on()
        self.mc.calendar = [daily('14:00')]

    def kinds(self):
        return [(j['kind'], j['code']) for j in self.journal()]

    def test_settings_with_who(self):
        self.set(by='owner', resume_max=5)
        j = self.journal()[-1]
        self.assertEqual((j['kind'], j['vars']['by'], j['vars']['values']),
                         ('settings', 'owner', {'resume_max': 5}))

    def test_a_run(self):
        mc = self.mc
        run = mc._new_run(mc.calendar[0], self.clock[0])
        mc.persistent['run'] = run
        mc._confirm_started('Mow')
        mc._handle_program_end('interrupted:temp', True)
        mc.persistent['run']['wait_until'] = 0
        mc._handle_program_end('succeeded', True)
        k = self.kinds()
        self.assertIn(('started', 'started'), k)
        self.assertIn(('waited', 'temp'), k)
        self.assertIn(('finished', 'succeeded'), k)

    def test_slept_and_woke(self):
        mc = self.mc
        mc.sleep_seq.start = lambda *a: None
        mc._execute_sleep(duration_ms=3600000)
        self.assertIn(('slept', 'timed'), self.kinds())

    def test_survives_restart_and_is_bounded(self):
        mc = self.mc
        for i in range(450):
            mc._journal('settings', 'changed', n=i)
        path = os.path.join(_HOME, '.vitulus', 'autonomy_journal.jsonl')
        with open(path) as f:
            self.assertLessEqual(len(f.readlines()), 401)
        self.assertEqual(len(mc.journal.entries()), 100)
        mc = self.make()
        self.assertEqual(len(mc.journal.entries()), 100)
        self.assertEqual(mc.journal.entries()[-1]['vars']['n'], 449)

    def test_cut_lines_are_archived_not_lost(self):
        mc = self.mc
        for i in range(450):
            mc._journal('settings', 'changed', n=i)
        vit = os.path.join(_HOME, '.vitulus')
        archived = []
        for path in sorted(glob.glob(os.path.join(vit, 'autonomy_journal.archive', '*.jsonl'))):
            self.assertRegex(os.path.basename(path), r'^\d{4}-\d{2}\.jsonl$')
            with open(path) as f:
                archived += [json.loads(l) for l in f]
        with open(os.path.join(vit, 'autonomy_journal.jsonl')) as f:
            live = [json.loads(l) for l in f]
        ns = [j['vars']['n'] for j in archived + live if 'n' in j['vars']]
        self.assertEqual(ns, list(range(450)))
        self.assertTrue(archived)

    def test_torn_line(self):
        path = os.path.join(_HOME, '.vitulus', 'autonomy_journal.jsonl')
        self.mc._journal('settings', 'changed')
        with open(path, 'a') as f:
            f.write('{"t": "2026')
        self.assertEqual(len(self.make().journal.entries()), 1)


class JournalArchive(unittest.TestCase):
    """autonomy.Journal cut-back: nothing lost, nothing archived twice."""

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='journal_')
        self.path = os.path.join(self.dir, 'autonomy_journal.jsonl')
        self.logs = []

    def tearDown(self):
        os.chmod(self.dir, 0o755)
        shutil.rmtree(self.dir, ignore_errors=True)

    def make(self):
        return A.Journal(self.path, log=self.logs.append)

    def item(self, n):
        return {'t': '2026-11-%02dT10:00:%02d' % (1 + n // 300, n % 60),
                'kind': 'k', 'code': 'c', 'vars': {'n': n}, 'program': ''}

    def ns(self):
        out = []
        for p in sorted(glob.glob(os.path.join(self.dir, 'autonomy_journal.archive', '*.jsonl'))):
            with open(p, errors='replace') as f:
                for l in f:
                    try:
                        out.append(json.loads(l)['vars']['n'])
                    except ValueError:
                        pass  # a line a crash tore
        with open(self.path, errors='replace') as f:
            for l in f:
                try:
                    out.append(json.loads(l)['vars']['n'])
                except ValueError:
                    out.append(l.strip())
        return out

    def fill(self, j, a, b):
        for n in range(a, b):
            j.add(self.item(n))

    def test_months_and_exactly_once(self):
        j = self.make()
        self.fill(j, 0, 900)
        self.assertEqual(self.ns(), list(range(900)))
        self.assertEqual(sorted(os.listdir(j.archive_dir)), ['2026-11.jsonl'])

    def test_crash_after_archive_before_rewrite(self):
        j = self.make()
        self.fill(j, 0, 400)
        real = os.replace
        with mock.patch.object(A.os, 'replace', side_effect=OSError('power cut')):
            j.add(self.item(400))  # archived, not cut, line appended
        self.assertTrue(any('rewrite' in m for m in self.logs))
        j = self.make()            # restart
        self.fill(j, 401, 450)
        self.assertIs(A.os.replace, real)
        self.assertEqual(self.ns(), list(range(450)))

    def test_crash_in_the_middle_of_archiving(self):
        j = self.make()
        self.fill(j, 0, 400)
        with open(self.path) as f:
            lines = f.read().splitlines()
        os.makedirs(j.archive_dir)
        with open(os.path.join(j.archive_dir, '2026-11.jsonl'), 'w') as f:
            f.write('\n'.join(lines[:120]) + '\n' + lines[120][:25])  # torn
        self.make().add(self.item(400))
        ns = [n for n in self.ns() if isinstance(n, int)]
        self.assertEqual(ns, list(range(401)))

    def test_archive_failure_does_not_cut(self):
        j = self.make()
        with open(j.archive_dir, 'w') as f:
            f.write('in the way')  # makedirs fails
        self.fill(j, 0, 410)
        self.assertTrue(any('archive' in m for m in self.logs))
        with open(self.path) as f:
            self.assertEqual(len(f.readlines()), 410)

    def test_read_failure_does_not_cut(self):
        j = self.make()
        self.fill(j, 0, 400)
        with mock.patch.object(A.Journal, '_archive', side_effect=AssertionError('no')):
            with mock.patch('builtins.open', side_effect=[OSError('eio'), open(self.path, 'a')]):
                j.add(self.item(400))
        self.assertTrue(any('read' in m for m in self.logs))
        self.assertEqual(self.ns(), list(range(401)))

    def test_bad_bytes_and_lines_missing_in_items(self):
        j = self.make()
        self.fill(j, 0, 300)
        with open(self.path, 'ab') as f:  # written by someone else, not in items
            f.write(b'{"t": "2026-11-02T00:00:00", "vars": {"n": "\xff"}}\n')
            for n in range(300, 399):
                f.write((json.dumps(self.item(n)) + '\n').encode())
        self.fill(j, 399, 420)
        ns = self.ns()
        self.assertEqual(len(ns), 421)
        self.assertEqual([n for n in ns if isinstance(n, int)], list(range(420)))


class Notify(Base):
    def setUp(self):
        super().setUp()
        self.on()
        self.mc.calendar = [daily('14:00')]

    def n(self):
        return [json.loads(m.data) for t, m in EVENTS if t == '/scheduler/notify']

    def test_missed(self):
        mc = self.mc
        mc.persistent['scan_from'] = '2026-06-09T00:00:00'
        mc.catchup_max_s = 3600
        self.clock[0] = D(2026, 6, 10, 16, 0, 0)
        mc.mission_busy = True
        mc._code = 'blocked_mission'
        mc._step()
        n = [x for x in self.n() if x['code'] == 'missed'][0]
        self.assertEqual((n['level'], n['program'], n['vars']['issue'], n['vars']['blocked_by']),
                         ('warn', 'Mow', 'slot_missed', 'blocked_mission'))
        self.assertIn('t', n)
        self.assertEqual(self.report()['issues'][-1]['level'], 'warn')
        self.assertIn(('missed', 'slot_missed'),
                      [(j['kind'], j['code']) for j in self.journal()])

    def test_stranded_and_failed(self):
        mc = self.mc
        mc.persistent['run'] = mc._new_run(mc.calendar[0], self.clock[0])
        mc._issue('run_stranded', 'Mow')
        mc._give_up(mc._run(), 'run_no_progress', reason='temp')
        self.assertEqual([x['code'] for x in self.n()], ['stranded', 'run_failed'])
        self.assertEqual(self.n()[1]['vars']['issue'], 'run_no_progress')
        self.assertEqual(self.n()[1]['level'], 'error')

    def test_sleep_failed(self):
        mc = self.mc
        mc.state = mc.STATE_PREPARE_SLEEP
        seq = mc.sleep_seq
        seq.state = SM.IDLE
        seq.result = 'pm_no_ack'
        mc._handle_prepare_sleep()
        self.assertEqual(self.n()[0]['code'], 'sleep_failed')
        self.assertEqual(self.n()[0]['vars']['reason'], 'pm_no_ack')

    def test_a_chosen_stop_is_no_news(self):
        mc = self.mc
        mc.state = mc.STATE_PREPARE_SLEEP
        mc.sleep_seq.state = SM.IDLE
        mc.sleep_seq.result = 'off_dock'
        mc._handle_prepare_sleep()
        self.assertEqual(self.n(), [])


class Review(Base):
    """Findings of the review, each reproduced here first."""

    def setUp(self):
        super().setUp()
        self.on()
        self.mc.calendar = [daily('14:00')]
        self.mc.window_enabled = False
        self.mc._wall = lambda text: self.walls.append(text)
        self.walls = []

    def selects(self):
        return [m.data for t, m in EVENTS if t == '/web_plan/program_select']

    def pm(self, armed=False, charged=False):
        self.mc.pm_status = types.SimpleNamespace(
            sleep_status=armed, sleep_charged_status=charged, battery_capacity=90,
            sleep_wait_standby=0, sleep_time_interval=0, sleep_wait_charged=0,
            charger_status='CHARGING', supply_status='ONLINE')
        self.mc.pm_status_time = self.mc._t()

    # H1 ------------------------------------------------------------------
    def test_h1_once_never_reaches_a_calendar_run(self):
        R = M.rain_rule
        self.assertTrue(R.checks_active('always', True, True))
        self.assertTrue(R.checks_active('calendar', True, True))
        self.assertFalse(R.checks_active('always', False, True))
        self.assertFalse(R.checks_active('ignore', True, False))

    def test_h1_calendar_start_disarms_it(self):
        mc = self.mc
        mc.ignore_rain_callback(Bool(True))
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._step()
        self.assertEqual(self.selects(), ['Mow'])
        self.assertIs(last('/scheduler/ignore_rain_once'), False)
        self.assertIsNone(mc._ignore_rain)

    def test_h1_rain_check_of_a_calendar_start_stays_on(self):
        self.mc.weather_msg = alert(0, self.mc._t(), rain_now=1)
        self.mc.ignore_rain_callback(Bool(True))
        self.assertEqual(self.check('rain')['code'], 'rain')
        self.assertIsNone(self.report()['rain_would_refuse'])

    # H2 ------------------------------------------------------------------
    def test_h2_catchup_floor(self):
        self.assertEqual(self.set(catchup_max_h=0)['rejected'], {'catchup_max_h': 'range'})
        self.assertEqual(self.set(catchup_max_h=0.4)['rejected'], {'catchup_max_h': 'range'})

    def test_h2_catchup_covers_a_wake(self):
        self.mc.boot_grace_period_s = 300.0
        self.assertEqual(self.set(catchup_max_h=0.5, wake_before_min=30.0)['rejected'],
                         {'catchup_max_h': 'catchup_under_wake'})
        self.assertTrue(self.set(catchup_max_h=0.5, wake_before_min=20.0)['ok'])

    # H3 ------------------------------------------------------------------
    def test_h3_nothing_starts_over_an_armed_pm(self):
        mc = self.mc
        self.pm(armed=True)
        self.assertEqual(self.check('mission')['code'], 'blocked_pm_armed')
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._step()
        self.assertEqual(self.selects(), [])
        self.assertEqual(mc._code, 'blocked_pm_armed')
        self.pm(charged=True)
        self.assertEqual(self.check('mission')['code'], 'blocked_pm_armed')
        self.pm()
        mc._step()
        self.assertEqual(self.selects(), ['Mow'])

    def test_h3_an_unconfirmed_cancel_goes_on(self):
        mc = self.mc
        mc.state = mc.STATE_PREPARE_SLEEP
        mc.sleep_seq.state = SM.IDLE
        mc.sleep_seq.result = 'pm_cancel_failed'
        self.pm(armed=True)
        mc._handle_prepare_sleep()
        del EVENTS[:]
        mc._step()
        self.assertIs(last('/pm/set_robot_sleep'), False)
        self.assertTrue(self.make().persistent.get('pm_cancel_pending'))
        self.clock[0] += dt.timedelta(seconds=10)
        self.pm()
        mc._step()
        self.assertFalse(mc.persistent.get('pm_cancel_pending'))
        del EVENTS[:]
        self.pm(armed=True)                     # now a person's raw sleep: not ours
        self.clock[0] += dt.timedelta(seconds=10)
        mc._step()
        self.assertIsNone(last('/pm/set_robot_sleep'))

    # M1 ------------------------------------------------------------------
    def test_m1_skip_during_a_run_is_not_the_running_slot(self):
        mc = self.mc
        self.clock[0] = D(2026, 6, 10, 14, 0, 30)
        mc._step()
        self.assertEqual(self.selects(), ['Mow'])
        self.assertTrue(self.set(skip_next=True)['ok'])
        self.assertEqual(mc.settings['skip_next'], '2026-06-11T14:00')
        self.assertEqual(lastj('/scheduler/settings_result')['skip_next'], '2026-06-11T14:00')

    # M2 ------------------------------------------------------------------
    def test_m2_someone_arriving_in_the_sequence_gets_a_countdown(self):
        mc = self.mc
        self.set(sleep_ignore_ssh=True)
        del mc.sleep_seq.start
        mc.sleep_seq.helper = types.SimpleNamespace(poll=lambda: SM.RC_SOFT)
        self.pm()
        mc._execute_sleep(duration_ms=3600000)
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)
        self.logins = ['pts/7']
        for _ in range(12):
            self.clock[0] += dt.timedelta(seconds=1)
            self.pm()
            mc._step()
            if mc._countdown:
                break
        self.assertIsNotNone(mc._countdown)
        self.assertEqual(lastj('/scheduler/sleep_countdown')['who']['ssh'], ['pts/7'])
        self.assertEqual(mc._sleep_cooldown_until, 0.0)

    def test_m2_sequence_keeps_saying_when_the_cut_comes(self):
        mc = self.mc
        self.set(sleep_ignore_ssh=True)
        self.logins = ['pts/1']
        mc._execute_sleep(duration_ms=3600000)
        for _ in range(60):
            mc._tick_sleep_requests()
        mc.sleep_seq.state = SM.WAIT_CUT
        mc.sleep_seq.helper = types.SimpleNamespace(poll=lambda: SM.RC_SOFT)
        mc.sleep_seq.clock = mc._t
        mc.sleep_seq.armed_at = mc._t()
        mc._handle_prepare_sleep()
        c = lastj('/scheduler/sleep_countdown')
        self.assertTrue(c['active'])
        self.assertEqual(c['left_s'], 180)
        mc.sleep_seq.helper = types.SimpleNamespace(poll=lambda: None)   # past return
        mc._handle_prepare_sleep()
        self.assertFalse(lastj('/scheduler/sleep_countdown')['active'])

    # M3 ------------------------------------------------------------------
    def test_m3_used_slot_is_not_next_and_not_run_twice(self):
        mc = self.mc
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        mc._confirm_started('Mow')
        mc._clear_interrupted()
        mc.state = mc.STATE_IDLE
        self.assertEqual(self.report()['next']['at'], '2026-06-11T14:00')
        mc.run_next_callback(Bool(True))
        mc._tick_sleep_requests()
        self.assertEqual(mc._run()['slot'], '2026-06-11T14:00')
        slots = lastj('/scheduler/timeline')['starts']
        self.assertEqual((slots[0]['state'], slots[0]['reason']), ('skip', 'already_run'))

    # M4 ------------------------------------------------------------------
    def countdown(self):
        self.set(sleep_ignore_ssh=True)
        self.logins = ['pts/1']
        self.mc._execute_sleep(duration_ms=3600000)
        self.assertIsNotNone(self.mc._countdown)

    def test_m4_changes_end_a_countdown(self):
        for values in ({'window_end': '20:00'}, {'wake_before_min': 5.0},
                       {'skip_next': True}, {'no_mow_until': '2026-06-10'}):
            self.countdown()
            self.assertTrue(self.set(**values)['ok'])
            self.mc._tick_sleep_requests()
            self.assertIsNone(self.mc._countdown, values)
            self.set(reset=list(values))
            self.mc._sleep_replan = False

    def test_m4_calendar_edit_ends_a_countdown_same_calendar_does_not(self):
        mc = self.mc
        self.countdown()
        mc.set_calendar_callback(String(json.dumps(mc.calendar)))
        mc._tick_sleep_requests()
        self.assertIsNotNone(mc._countdown)
        mc.set_calendar_callback(String(json.dumps([daily('15:00')])))
        mc._tick_sleep_requests()
        self.assertIsNone(mc._countdown)

    # M5 ------------------------------------------------------------------
    def test_m5_battery_bounds(self):
        self.assertEqual(self.set(battery_return_pct=19)['rejected'],
                         {'battery_return_pct': 'range'})
        self.assertEqual(self.set(battery_return_pct=36)['rejected'],
                         {'battery_start_pct': 'battery_start_under_return'})
        self.assertTrue(self.set(battery_return_pct=35)['ok'])
        self.assertEqual(self.set(battery_start_pct=81, battery_resume_pct=90)['rejected'],
                         {'battery_start_pct': 'range'})

    # M6 ------------------------------------------------------------------
    def test_m6_no_predicate_no_green(self):
        self.assertEqual(self.check('gnss')['state'], 'unknown')
        self.assertEqual(self.check('temperature')['state'], 'unknown')

    # L -------------------------------------------------------------------
    def test_l1_report_stays_small(self):
        mc = self.mc
        for i in range(100):
            mc._journal('settings', 'changed', by='owner', values={'resume_max': i})
        rep = self.report()
        self.assertNotIn('journal', rep)
        self.assertNotIn('timeline', rep)
        self.assertLess(len(last('/scheduler/report')), 6000)
        self.assertEqual(len(lastj('/scheduler/journal')['entries']), 100)
        self.assertEqual(len(lastj('/scheduler/timeline')['starts']), 7)

    def test_l2_next_event_stops_at_the_first_day(self):
        calls = []
        orig = M.MowerScheduler._event_runs_on.__func__

        def counted(cls, ev, day):
            calls.append(day)
            return orig(cls, ev, day)

        M.MowerScheduler._event_runs_on = classmethod(counted)
        try:
            M.MowerScheduler.get_next_event([daily('14:00')], self.clock[0])
        finally:
            M.MowerScheduler._event_runs_on = classmethod(orig)
        self.assertEqual(len(calls), 1)

    def test_l3_save_failure_says_so_and_changes_nothing(self):
        mc = self.mc

        def fail(doc):
            raise OSError('disk')

        mc.settings_store.save = fail
        r = self.set(temp_wait_h=2.0)
        self.assertEqual((r['ok'], r['code']), (False, 'save'))
        self.assertEqual(mc.settings['temp_wait_h'], 1.0)
        self.assertEqual(mc.temp_sleep_duration_s, 3600.0)

    def test_l4_reset_of_junk(self):
        r = self.set(reset=[['x'], {'a': 1}, 3])
        self.assertFalse(r['ok'])

    def test_l7_window_shorter_than_the_margin(self):
        self.assertEqual(self.set(window_start='12:00', window_end='12:20')['rejected'],
                         {'window_min_left_min': 'window_too_short'})

    def test_l8_ssh_person_is_told(self):
        self.countdown()
        self.assertEqual(len(self.walls), 1)
        self.assertIn('60', self.walls[0])


if __name__ == '__main__':
    unittest.main()
