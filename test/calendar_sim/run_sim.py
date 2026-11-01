#!/usr/bin/env python3
"""Full calendar cycle off the robot: real master_controller + real
mower_unit_smach + fake_world.py on a PRIVATE roscore (port 11811+).

  python3 run_sim.py                 all scenarios, this tree
  python3 run_sim.py normal rain     some of them
  python3 run_sim.py --old           against git HEAD (the code before the work)

Nothing here touches the live robot: every process gets its own ROS_MASTER_URI,
ROS_HOME and HOME; only processes started here are ever signalled.
Time: each scenario sets a fake local clock for master_controller (TZ
Europe/Prague) and jumps it; everything else runs in real time.
"""
import argparse
import datetime as dt
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
PLANNER = os.path.abspath(os.path.join(HERE, '..', '..'))
SCENARIOS = ['disabled', 'normal', 'prestart_rain', 'battery', 'rain', 'rain_reject_keeps',
             'stack_restart', 'master_restart', 'mission_crash', 'dst', 'window_end',
             'terminal_cap', 'stop_visible', 'agent_block', 'no_weather', 'mux_hold', 'hold_rain',
             'window_change', 'window_back', 'window_off', 'manual_rain', 'manual_rain_calendar']


def navi_tree():
    """vitulus_navi of the same worktree name, or env SIM_NAVI."""
    if os.environ.get('SIM_NAVI'):
        return os.environ['SIM_NAVI']
    # <vitulus>/vitulus_planner/.claude/worktrees/<name>
    parts = PLANNER.split(os.sep)
    i = parts.index('vitulus_planner')
    return os.sep.join(parts[:i] + ['vitulus_navi'] + parts[i + 1:])


def export_old(tmp):
    root = os.sep.join(PLANNER.split(os.sep)[:PLANNER.split(os.sep).index('vitulus_planner')])
    out = {}
    for repo, key in (('vitulus_planner', 'planner'), ('vitulus_navi', 'navi')):
        d = os.path.join(tmp, repo)
        os.makedirs(d)
        ps = subprocess.Popen(['git', '-C', os.path.join(root, repo), 'archive', 'HEAD',
                               'nodes', 'src', 'config'], stdout=subprocess.PIPE)
        subprocess.check_call(['tar', '-x', '-C', d], stdin=ps.stdout)
        ps.wait()
        out[key] = d
    return out


def port_base():
    """Two checkouts running the simulation at once must not share a roscore."""
    if os.environ.get('SIM_PORT_BASE'):
        return int(os.environ['SIM_PORT_BASE'])
    return 11811 + (zlib.crc32(PLANNER.encode()) % 8) * 20


def main_outer():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenarios', nargs='*')
    ap.add_argument('--old', action='store_true')
    ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()
    names = a.scenarios or SCENARIOS
    tmp = tempfile.mkdtemp(prefix='calsim_')
    trees = {'planner': PLANNER, 'navi': navi_tree()}
    if a.old:
        trees = export_old(tmp)
    results = {}
    for i, name in enumerate(names):
        env = dict(os.environ, SIM_INNER='1', SIM_PORT=str(port_base() + i), SIM_TMP=os.path.join(tmp, name),
                   SIM_PLANNER=trees['planner'], SIM_NAVI_TREE=trees['navi'],
                   SIM_OLD='1' if a.old else '', TZ='Europe/Prague')
        os.makedirs(env['SIM_TMP'])
        t0 = time.time()
        rc = subprocess.call([sys.executable, os.path.abspath(__file__), name], env=env)
        results[name] = rc
        print('=== %-20s %s (%.0f s)' % (name, 'PASS' if rc == 0 else 'FAIL', time.time() - t0), flush=True)
    print('\nSUMMARY (%s):' % ('old code' if a.old else 'this tree'))
    for n, rc in results.items():
        print('  %-20s %s' % (n, 'PASS' if rc == 0 else 'FAIL'))
    if not a.keep and all(r == 0 for r in results.values()):
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        print('logs:', tmp)
    return 0 if all(r == 0 for r in results.values()) else 1


if not os.environ.get('SIM_INNER'):
    sys.exit(main_outer())

# ---------------------------------------------------------------------------
# one scenario (own process, own roscore)
# ---------------------------------------------------------------------------
PORT = os.environ['SIM_PORT']
TMP = os.environ['SIM_TMP']
OLD = bool(os.environ['SIM_OLD'])
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%s' % PORT
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)
os.environ['ROS_HOME'] = os.path.join(TMP, 'ros')
HOME = os.path.join(TMP, 'home')
os.makedirs(HOME, exist_ok=True)
os.environ['SIM_LOG'] = os.path.join(TMP, 'events.jsonl')
os.environ['SIM_STORE'] = os.path.join(TMP, 'world.json')
os.environ['SIM_CLOCK'] = os.path.join(TMP, 'clock')
open(os.environ['SIM_CLOCK'], 'w').write('0')
open(os.environ['SIM_LOG'], 'w').close()

import rospy                                         # noqa: E402
from std_msgs.msg import Bool, Empty, String, UInt64  # noqa: E402,F401

STATE_FILE = os.path.join(HOME, '.vitulus', 'scheduler_state.json')
PLANNER_T = os.environ['SIM_PLANNER']
NAVI_T = os.environ['SIM_NAVI_TREE']
FAILS = []


def check(name, cond, detail=''):
    print('  %s %s %s' % ('ok  ' if cond else 'FAIL', name, detail), flush=True)
    if not cond:
        FAILS.append(name)


class Sim(object):
    def __init__(self, master_params=None, smach_params=None, enabled=True):
        self.procs = {}
        self.master_params = master_params or {}
        self.smach_params = smach_params or {}
        self.enabled = enabled
        self.logdir = os.path.join(TMP, 'logs')
        os.makedirs(self.logdir)
        self.spawn('core', ['roscore', '-p', PORT, '--master-logger-level', 'warn'])
        for _ in range(100):
            if subprocess.call(['rosparam', 'list'], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) == 0:
                break
            time.sleep(0.2)
        rospy.init_node('sim_driver', disable_signals=True)
        self.status = ''
        self.stop_reason = ''
        self.sched = ''
        self.report = {}
        self.enabled_seen = None
        self.sleep_time = 0
        self.until_charged = False
        self.sleep_req = False
        self.busy = False
        rospy.Subscriber('/mower_smach/status', String, self._s('status'))
        rospy.Subscriber('/mower_smach/stop_reason', String, self._s('stop_reason'))
        rospy.Subscriber('/scheduler/status', String, self._s('sched'))
        rospy.Subscriber('/scheduler/report', String, self._report)
        rospy.Subscriber('/scheduler/enabled', Bool, self._s('enabled_seen'))
        rospy.Subscriber('/mower_smach/busy', Bool, self._s('busy'))
        rospy.Subscriber('/pm/set_sleep_time', UInt64, self._s('sleep_time'))
        rospy.Subscriber('/pm/set_sleep_until_charged', Bool, self._s('until_charged'))
        rospy.Subscriber('/pm/set_robot_sleep', Bool, self._s('sleep_req'))
        self.p_set = rospy.Publisher('/sim/set', String, queue_size=5)
        self.p_cal = rospy.Publisher('/scheduler/set_calendar', String, queue_size=1, latch=True)
        self.p_en = rospy.Publisher('/scheduler/set_enabled', Bool, queue_size=1, latch=True)
        self.p_stop = rospy.Publisher('/mower_smach/stop', Bool, queue_size=1)
        self.p_reset = rospy.Publisher('/mower_smach/reset', Bool, queue_size=1)
        self.p_agent = rospy.Publisher('/vitulus_test/supervisor_status', String, queue_size=1)
        self.p_mux = rospy.Publisher('/cmd_vel_mux/active', String, queue_size=1, latch=True)
        self.p_window = rospy.Publisher('/scheduler/set_window', String, queue_size=1)
        self.p_held = rospy.Publisher('/cmd_vel_mux/held', Bool, queue_size=1, latch=True)
        self.load_params()
        self.spawn_world()
        self.spawn_smach()
        self.spawn_master()
        self.wait(lambda: self.status == 'Ready', 40, 'smach up')

    def _s(self, attr):
        def cb(msg):
            setattr(self, attr, msg.data)
        return cb

    def _report(self, msg):
        self.report = json.loads(msg.data)

    def spawn(self, key, cmd, env=None):
        log = open(os.path.join(self.logdir, key + '.log'), 'ab')
        e = dict(os.environ, HOME=HOME)
        if env:
            e.update(env)
        self.procs[key] = subprocess.Popen(cmd, env=e, stdout=log, stderr=log,
                                           preexec_fn=os.setsid)

    def kill(self, key, sig=signal.SIGKILL):
        p = self.procs.pop(key, None)
        if p is not None:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
            p.wait()

    def load_params(self):
        def rp(*a):
            subprocess.check_call(('rosparam',) + a)
        rp('load', os.path.join(PLANNER_T, 'config', 'master_controller.yaml'), '/master_controller')
        rp('load', os.path.join(NAVI_T, 'config', 'mower_smach.yaml'), '/node_mower_smach')
        base = {'boot_grace_period_s': 2.0, 'startup_delay_s': 1.0, 'check_interval_s': 1.0,
                'tick_hz': 4.0, 'mission_lost_grace_s': 4.0, 'off_dock_wait_s': 100000.0,
                'sleep_wait_before_standby_s': 1.0, 'sleep_min_off_s': 0.0, 'sleep_min_s': 1.0,
                'sleep_timer_drift': 0.0, 'sleep_final_grace_s': 0.2, 'sleep_retry_s': 0.6,
                'sleep_check_logins': False, 'sleep_countdown_s': 0.0, 'sleep_shutdown_cmd': '/bin/true',
                'automation_enabled': self.enabled if OLD else False}
        base.update(self.master_params)
        for k, v in base.items():
            rospy.set_param('/master_controller/' + k, v)
        for k, v in self.smach_params.items():
            rospy.set_param('/node_mower_smach/' + k, v)

    def spawn_world(self):
        self.spawn('world', [sys.executable, os.path.join(HERE, 'fake_world.py')])

    def spawn_smach(self):
        env = {'PYTHONPATH': os.path.join(NAVI_T, 'src') + ':' + os.environ.get('PYTHONPATH', '')}
        self.spawn('smach', [sys.executable, os.path.join(NAVI_T, 'nodes', 'mower_unit_smach'),
                             '__name:=node_mower_smach'], env)

    def spawn_master(self):
        env = {'PYTHONPATH': os.path.join(NAVI_T, 'src') + ':' + os.environ.get('PYTHONPATH', '')}
        self.spawn('master', [sys.executable, os.path.join(HERE, 'sim_master.py'),
                              os.path.join(PLANNER_T, 'nodes', 'master_controller'),
                              '__name:=master_controller'], env)

    def close(self):
        for k in list(self.procs):
            if k != 'core':
                self.kill(k)
        self.kill('core', signal.SIGINT)
        rospy.signal_shutdown('done')

    # --- time ---------------------------------------------------------------
    def set_clock(self, local):
        off = time.mktime(local.timetuple()) - time.time()
        open(os.environ['SIM_CLOCK'], 'w').write(repr(off))

    def now(self):
        off = float(open(os.environ['SIM_CLOCK']).read())
        return dt.datetime.fromtimestamp(time.time() + off)

    def jump(self, seconds):
        off = float(open(os.environ['SIM_CLOCK']).read())
        open(os.environ['SIM_CLOCK'], 'w').write(repr(off + seconds))

    # --- world --------------------------------------------------------------
    def set(self, **kw):
        for _ in range(3):
            self.p_set.publish(String(json.dumps(kw)))
            time.sleep(0.1)

    def events(self, kind=None):
        out = []
        with open(os.environ['SIM_LOG']) as f:
            for line in f:
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if kind is None or e['kind'] == kind:
                    out.append(e)
        return out

    def mowed(self):
        """Mowing chunks in order, as (path y, first x)."""
        return [(e['first'][1], e['first'][0]) for e in self.events('exe')
                if not e['transit'] and e['blade']]

    def selects(self):
        return self.events('select')

    def state(self):
        try:
            return json.load(open(STATE_FILE))
        except (OSError, ValueError):
            return {}

    def calendar(self, *events):
        self.wait(lambda: self.p_cal.get_num_connections() > 0, 15, 'master listens')
        self.p_cal.publish(String(json.dumps(list(events))))
        time.sleep(0.3)

    def enable(self, on=True):
        self.wait(lambda: self.p_en.get_num_connections() > 0, 15, 'master listens')
        self.p_en.publish(Bool(on))
        time.sleep(0.3)

    def stack_restart(self, world=True):
        self.kill('master')
        self.kill('smach')
        if world:
            self.kill('world')
            self.spawn_world()
            time.sleep(1.5)
        self.status = ''
        self.spawn_smach()
        self.spawn_master()
        self.wait(lambda: self.status == 'Ready', 40, 'smach back')

    def serve_sleep(self):
        """Power module: the robot sleeps, everything dies, the clock moves on
        by the real sleep length, the battery charges, everything boots."""
        ms = self.sleep_time or 3 * 3600 * 1000
        if self.until_charged:
            ms = max(ms if self.sleep_time else 0, 3 * 3600 * 1000)
        # the fake bench keeps time exactly: the master's number is what counts
        with open(os.environ['SIM_LOG'], 'a') as f:
            f.write(json.dumps({'kind': 'sleep', 'ms': ms, 'charged': self.until_charged,
                                'at': self.now().isoformat(), 't': time.time()}) + '\n')
        w = json.load(open(os.environ['SIM_STORE']))
        self.set(battery=min(100.0, w['s']['battery'] + ms / 3600000.0 * 30.0))
        time.sleep(0.2)
        self.sleep_req = False
        self.sleep_time = 0
        self.until_charged = False
        self.kill('master')
        self.kill('smach')
        self.kill('world')
        self.jump(ms / 1000.0)
        self.spawn_world()
        time.sleep(1.5)
        self.status = ''
        self.spawn_smach()
        self.spawn_master()
        self.wait(lambda: self.status == 'Ready', 40, 'smach after sleep', serve=False)

    def wait(self, cond, timeout, label, serve=True):
        end = time.time() + timeout
        while time.time() < end:
            if serve and self.sleep_req:
                self.serve_sleep()
            if cond():
                return True
            time.sleep(0.05)
        print('  (timeout: %s; status=%r sched=%r reason=%r)' % (
            label, self.status, self.sched, self.stop_reason), flush=True)
        return False


def daily(t, eid='ev1', program='Mow'):
    return {'id': eid, 'program_name': program, 'enabled': True, 'recurrence': 'daily',
            'start_time': t}


D = dt.datetime


def start(s, clock, *events, enable=True):
    s.set_clock(clock)
    if OLD:
        # The old node has no switch and measures its boot delay on the clock
        # that was just moved: give it a boot on the new clock instead.
        s.kill('master')
        s.spawn_master()
    s.calendar(*events)
    if enable and not OLD:
        s.enable()


def finish_ok(s, timeout=120):
    return s.wait(lambda: s.stop_reason == 'succeeded' and s.status == 'Ready'
                  and json.load(open(os.environ['SIM_STORE']))['s']['docked'], timeout, 'finished')


ALL_PATHS = [0.0, 1.0, 2.0, 10.0, 11.0, 12.0]


def scenario_disabled():
    s = Sim(enabled=False)
    start(s, D(2026, 6, 10, 7, 59, 50), daily('08:00'), enable=False)
    time.sleep(16)
    check('no start with the switch off', not s.selects())
    s.set_clock(D(2026, 6, 10, 22, 0, 0))
    time.sleep(6)
    check('no sleep with the switch off (outside window, empty run)', not s.sleep_req)
    check('report says off', s.report.get('enabled') is False and s.report.get('code') == 'off', s.report.get('code'))
    # On at 22:00: slot of 08:00 is not caught up, calendar waits for tomorrow
    s.enable(True)
    s.wait(lambda: s.enabled_seen is True, 5, 'enabled echo')
    check('switch echoes on', s.enabled_seen is True)
    check('persisted', s.state().get('automation_enabled') is True)
    s.kill('master')
    s.enabled_seen = None
    s.spawn_master()
    s.wait(lambda: s.enabled_seen is not None, 15, 'enabled after restart')
    check('survives restart', s.enabled_seen is True)
    s.enable(False)
    s.wait(lambda: s.enabled_seen is False, 5, 'enabled echo')
    s.sleep_req = False
    s.sleep_time = 0
    time.sleep(5)
    check('off again: no sleep, no start', not s.sleep_req and not s.selects())
    return s


def scenario_normal():
    s = Sim()
    start(s, D(2026, 6, 10, 7, 59, 54), daily('08:00'))
    check('undock order', s.wait(lambda: s.selects(), 25, 'select'))
    st = s.state()
    check('last_run not written before the real start',
          OLD or not st.get('last_run_by_event'), st.get('last_run_by_event'))
    check('finished', finish_ok(s))
    check('every path mowed once, in order',
          sorted(set(y for y, _ in s.mowed())) == ALL_PATHS, sorted(set(y for y, _ in s.mowed())))
    check('last_run written', s.state().get('last_run_by_event', {}).get('ev1') == '2026-06-10')
    time.sleep(8)
    check('no second start', len(s.selects()) == 1, len(s.selects()))
    return s


def scenario_prestart_rain():
    """Raining at the slot: nothing moves, last_run stays empty, it starts once
    it is dry (a fresh start, not a resume of some old checkpoint)."""
    s = Sim(master_params={'weather_sleep_duration_h': 0.002, 'idle_sleep_threshold_s': 1e7})
    s.wait(lambda: os.path.exists(os.environ['SIM_STORE']), 10, 'store')
    s.set(rain_now=1)
    time.sleep(1.5)
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('refused', s.wait(lambda: s.stop_reason == 'rejected:rain', 30, 'rejected'), s.stop_reason)
    check('never undocked', not s.events('dock_program'))
    check('last_run empty', not s.state().get('last_run_by_event', {}).get('ev1'))
    s.set(rain_now=0)
    check('started after the rain', s.wait(lambda: len(s.events('dock_program')) >= 1, 40, 'undock'))
    check('fresh start', s.selects() and not s.selects()[-1]['resume'], s.selects()[-1:])
    check('finished', finish_ok(s))
    check('last_run written', s.state().get('last_run_by_event', {}).get('ev1') == '2026-06-10')
    return s


def scenario_battery():
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing path 1', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    s.set(battery=24.0)
    check('interrupted', s.wait(lambda: s.stop_reason == 'interrupted:battery', 20, 'battery'), s.stop_reason)
    n1 = len(s.mowed())
    y1, x1 = s.mowed()[-1]
    check('back in the dock', s.wait(lambda: len(s.events('dock_program')) >= 2 and s.status == 'Ready', 40, 'dock'))
    time.sleep(1)
    wb = s.events('write_back')[-1]['last_result']
    check('checkpoint saved at once, not a minute later', wb.endswith('-%d' % int(wb.rsplit('-', 1)[1])) and cp_y(wb) == y1, wb)
    s.set(charge_rate=40.0)
    check('resumed', s.wait(lambda: len([e for e in s.selects() if e['resume']]) >= 1, 90, 'resume'))
    sel = [e for e in s.selects() if e['resume']][0]
    check('resume carries the checkpoint', cp_y(sel['last_result']) == y1, sel['last_result'])
    check('mowing again', s.wait(lambda: len(s.mowed()) > n1, 60, 'mow'))
    y2, x2 = s.mowed()[n1]
    check('resumed on the same path at the window', y2 == y1 and abs(x2 - x1) <= 0.5, (y1, x1, y2, x2))
    check('nothing earlier mowed again', all(y >= y1 for y, _ in s.mowed()[n1:]))
    # second time, in the other zone
    check('reaches ZoneB', s.wait(lambda: any(y == 10.0 for y, _ in s.mowed()), 90, 'zoneB'))
    s.wait(lambda: s.mowed()[-1][0] == 10.0 and abs(s.mowed()[-1][1]) > 1, 20, 'in zoneB')
    s.set(charge_rate=1.0)
    s.set(battery=24.0)
    check('interrupted again', s.wait(lambda: s.stop_reason == 'interrupted:battery', 20, 'battery2'))
    n2 = len(s.mowed())
    y3 = s.mowed()[-1][0]
    s.wait(lambda: s.status == 'Ready', 40, 'dock2')
    s.set(charge_rate=40.0)
    check('finished the program', finish_ok(s, 150))
    check('second resume kept the zone', all(y >= y3 for y, _ in s.mowed()[n2:]), s.mowed()[n2:][:2])
    check('every path done', sorted(set(y for y, _ in s.mowed())) == ALL_PATHS)
    check('two resumes, no fresh restart', len([e for e in s.selects() if not e['resume']]) == 1, len(s.selects()))
    return s


def cp_y(last_result):
    """Path line (y) of a checkpoint 'failed: on_path-ZoneA-1-200'."""
    _, zone, path, _w = last_result.rsplit('-', 3)
    return {'ZoneA': 0.0, 'ZoneB': 10.0}[zone] + int(path)


def _rain_run(s, forecast=False):
    """Rain on path 1, shared by the rain scenarios. forecast: also in the
    nowcast, which is all the old in-mission monitor looked at."""
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing path 1', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    s.set(rain_now=1, past_rain=True, forecast_rain=forecast)
    check('rain ends the mowing', s.wait(lambda: s.stop_reason == 'interrupted:weather', 20, 'weather'), s.stop_reason)
    check('back in the dock', s.wait(lambda: len(s.events('dock_program')) >= 2 and s.status == 'Ready', 40, 'dock'))
    time.sleep(1)
    wb = s.events('write_back')[-1]['last_result']
    check('checkpoint saved at the interruption', 'on_path' in wb, wb)
    return len(s.mowed()), cp_y(wb)


def _rain_rest(s, n1, y1):
    # first retry: raining stopped but still recent -> refused at the start
    s.set(rain_now=0, forecast_rain=False)
    check('retry refused (recent rain)', s.wait(lambda: s.stop_reason == 'rejected:recent_rain', 60, 'recent'), s.stop_reason)
    # the periodic save (60 s) must not eat the checkpoint meanwhile
    time.sleep(62)
    s.set(past_rain=False)
    check('finished after the rain', finish_ok(s, 150))
    check('resumed on the path it left', s.mowed()[n1][0] == y1, (y1, s.mowed()[n1:n1 + 1]))
    # (the wait above must have let the mission write nothing else over it)
    check('nothing earlier mowed again', all(y >= y1 for y, _ in s.mowed()[n1:]), [y for y, _ in s.mowed()[n1:]][:4])


def scenario_rain():
    """As shipped: rain_mode always. A calendar run obeys."""
    s = Sim(master_params={'weather_sleep_duration_h': 0.003, 'idle_sleep_threshold_s': 1e7})
    n1, y1 = _rain_run(s)
    _rain_rest(s, n1, y1)
    return s


def scenario_rain_reject_keeps():
    """rain_mode calendar, today's old default: a calendar run obeys all the same."""
    s = Sim(master_params={'weather_sleep_duration_h': 0.003, 'idle_sleep_threshold_s': 1e7,
                           'rain_mode': 'calendar'})
    n1, y1 = _rain_run(s, forecast=True)
    _rain_rest(s, n1, y1)
    return s


def scenario_stack_restart():
    """Stack restart while mowing, with the planner and master files on disk:
    the robot stands in the field. Nothing starts; when it is back in the dock
    the program resumes."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing path 1', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    time.sleep(0.5)
    n1 = len(s.mowed())
    y1 = s.mowed()[-1][0]
    s.stack_restart()
    time.sleep(8)
    check('no start off the dock', len(s.selects()) == 1)
    check('reported', OLD or s.report.get('code') == 'run_off_dock', s.report.get('code'))
    check('issue kept', OLD or any(i['code'] == 'run_stranded' for i in s.state().get('issues', [])))
    s.set(docked=True)
    check('finished after docking', finish_ok(s, 150))
    check('resume, not a fresh start', [e['resume'] for e in s.selects()] == [False, True], [e['resume'] for e in s.selects()])
    check('resumed at the checkpoint', s.mowed()[n1][0] == y1, s.mowed()[n1:n1 + 1])
    time.sleep(3)
    check('run closed at the end', s.state().get('run') is None, s.state().get('run'))
    return s


def scenario_master_restart():
    """Only master_controller restarts: the mission goes on. The finished
    program must not be started again."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    s.kill('master')
    time.sleep(1)
    s.spawn_master()
    check('finished', finish_ok(s))
    time.sleep(10)
    check('not started again', len(s.selects()) == 1, [e['resume'] for e in s.selects()])
    return s


def scenario_mission_crash():
    """mower_unit_smach dies mid-run with the robot in the field, the master
    lives. The run must not sit in "Mowing" for ever."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    s.kill('smach')
    s.status = ''
    s.spawn_smach()
    s.wait(lambda: s.status == 'Ready', 40, 'smach back')
    check('master noticed', s.wait(lambda: 'Mowing' not in s.sched, 20, 'lost'), s.sched)
    s.set(docked=True)
    check('finished after docking', finish_ok(s, 150))
    return s


def scenario_dst():
    """Saturday night before the spring change, slot Sunday 07:05: the sleep
    must be one hour shorter than the wall clock suggests, and the robot must
    be awake before the slot."""
    s = Sim()
    start(s, D(2027, 3, 27, 21, 30, 0), daily('07:05'))
    check('slept', s.wait(lambda: s.events('sleep'), 30, 'sleep'), s.sleep_time)
    s.wait(lambda: s.status == 'Ready', 40, 'boot')
    woke = s.now()
    check('awake before the slot', woke < D(2027, 3, 28, 7, 5), woke)
    if woke < D(2027, 3, 28, 7, 4, 50):
        s.set_clock(D(2027, 3, 28, 7, 4, 50))
    check('started', s.wait(lambda: s.selects(), 40, 'select'))
    check('at the slot', s.now() < D(2027, 3, 28, 7, 7), s.now())
    return s


def scenario_window_end():
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 20, 19, 56), daily('20:20'))
    check('mowing path 0', s.wait(lambda: any(y == 0.0 for y, _ in s.mowed()), 60, 'path 0'))
    s.set_clock(D(2026, 6, 10, 21, 0, 30))
    check('goes home', s.wait(lambda: s.stop_reason == 'interrupted:window', 30, 'window'), s.stop_reason)
    check('finished the path first', all(y == 0.0 for y, _ in s.mowed()), s.mowed())
    check('docked', s.wait(lambda: len(s.events('dock_program')) >= 2 and s.status == 'Ready', 40, 'dock'))
    time.sleep(5)
    check('run kept', OLD or (s.state().get('run') or {}).get('reason') == 'window', s.state().get('run'))
    check('no resume at night', len(s.selects()) == 1)
    s.set_clock(D(2026, 6, 11, 7, 0, 5))
    check('resumes in the next window', s.wait(lambda: len(s.selects()) >= 2, 40, 'morning'))
    check('as a resume', s.selects()[-1]['resume'] is True)
    check('morning', s.now().hour >= 7 and s.now().hour < 10, s.now())
    check('finished', finish_ok(s, 150))
    check('next path, not the same again', [y for y, _ in s.mowed()].count(0.0) >= 1 and sorted(set(y for y, _ in s.mowed())) == ALL_PATHS)
    return s


def set_window(s, **cfg):
    s.p_window.publish(String(json.dumps(cfg)))


def scenario_window_change():
    """The owner closes the window during mowing (real topic, real mission): the
    window-end path; the setting survives a restart; widening it again lets the
    run resume by the usual rules."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 9, 59, 56), daily('10:00'))
    check('mowing path 0', s.wait(lambda: any(y == 0.0 for y, _ in s.mowed()), 60, 'path 0'))
    set_window(s, enabled=True, start='07:00', end='09:00')
    check('reported at once', s.wait(lambda: (s.report.get('window') or {}).get('end') == '09:00', 5, 'report'),
          s.report.get('window'))
    check('stored', s.state().get('allowed_window') == {'enabled': True, 'start': '07:00', 'end': '09:00'},
          s.state().get('allowed_window'))
    check('goes home', s.wait(lambda: s.stop_reason == 'interrupted:window', 40, 'window'), s.stop_reason)
    check('finished the path first', all(y == 0.0 for y, _ in s.mowed()), s.mowed())
    check('docked', s.wait(lambda: len(s.events('dock_program')) >= 2 and s.status == 'Ready', 40, 'dock'))
    time.sleep(3)
    check('run kept', (s.state().get('run') or {}).get('reason') == 'window', s.state().get('run'))
    s.kill('master')
    s.report = {}
    s.spawn_master()
    check('survives a restart', s.wait(lambda: (s.report.get('window') or {}).get('end') == '09:00', 15, 'restart'),
          s.report.get('window'))
    time.sleep(3)
    check('no resume outside it', len(s.selects()) == 1, s.selects())
    set_window(s, enabled=True, start='07:00', end='21:00')
    check('resumes in the new window', s.wait(lambda: len(s.selects()) >= 2, 40, 'resume'))
    check('as a resume', s.selects()[-1]['resume'] is True)
    check('finished', finish_ok(s, 150))
    return s


def scenario_window_back():
    """Closed and opened again before the path ended: it just mows on."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 9, 59, 56), daily('10:00'))
    check('mowing path 0', s.wait(lambda: any(y == 0.0 for y, _ in s.mowed()), 60, 'path 0'))
    set_window(s, end='09:00')
    time.sleep(1.5)
    set_window(s, end='21:00')
    check('finished', finish_ok(s, 150))
    check('never went home early', s.stop_reason == 'succeeded' and len(s.selects()) == 1,
          (s.stop_reason, s.selects()))
    return s


def scenario_window_off():
    """With the switch off the setting is stored and reported, nothing moves."""
    s = Sim(enabled=False)
    s.set_clock(D(2026, 6, 10, 10, 0, 0))
    check('report', s.wait(lambda: s.report.get('enabled') is False, 20, 'report'))
    set_window(s, enabled=False, start='22:00', end='05:00')
    check('stored', s.wait(lambda: s.state().get('allowed_window') == {
        'enabled': False, 'start': '22:00', 'end': '05:00'}, 5, 'stored'), s.state().get('allowed_window'))
    check('reported', s.wait(lambda: s.report.get('window') is None and (
        s.report.get('window_setting') or {}).get('start') == '22:00', 5, 'report'), s.report)
    set_window(s, enabled=True, start='10:00', end='10:00')
    check('equal refused', s.wait(lambda: (s.report.get('window_setting') or {}).get('error') == 'equal', 5, 'refused'))
    check('still off', s.state().get('automation_enabled') is False and s.report.get('window') is None)
    time.sleep(3)
    check('nothing moved', not s.selects() and not s.sleep_req)
    return s


def scenario_terminal_cap():
    """The undock fails every time: TERMINAL in the dock. Visible, retried up
    to the cap, then a report, and it stops trying."""
    s = Sim(master_params={'terminal_error_short_wait_min': 0.05, 'critical_error_sleep_duration_h': 0.0005,
                           'critical_retry_max': 3, 'idle_sleep_threshold_s': 1e7})
    s.wait(lambda: os.path.exists(os.environ['SIM_STORE']), 10, 'store')
    s.set(undock_fail=True)
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('terminal shown', s.wait(lambda: s.status == 'TERMINAL ERROR', 40, 'terminal'))
    check('code', OLD or s.wait(lambda: s.report.get('code') == 'blocked_terminal', 6, 'code'), s.report.get('code'))
    check('gives up with a report', s.wait(
        lambda: any(i['code'] == 'run_failed' for i in s.state().get('issues', [])), 120, 'cap'),
        s.state().get('issues'))
    n = len(s.selects())
    time.sleep(10)
    check('stops trying', len(s.selects()) == n and n <= 3, n)
    return s


def scenario_stop_visible():
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing', s.wait(lambda: any(y == 0.0 for y, _ in s.mowed()), 60, 'path'))
    s.p_stop.publish(Bool(True))
    check('stopped', s.wait(lambda: s.status == 'Stopped', 20, 'stop'))
    check('report', s.wait(lambda: s.report.get('code') == 'blocked_stopped', 6, 'code'), s.report.get('code'))
    time.sleep(1)
    s.p_reset.publish(Bool(True))
    s.wait(lambda: s.status == 'Ready', 10, 'reset')
    time.sleep(8)
    check('no restart after a person stopped it', len(s.selects()) == 1, len(s.selects()))
    check('run closed', s.state().get('run') is None and s.state().get('interrupted_event_id') is None)
    return s


def scenario_agent_block():
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 54), daily('08:00'))
    busy = String(json.dumps({'state': 'RUNNING', 'run_id': 'agent_x', 'mow_program': None}))
    t0 = time.time()
    while time.time() - t0 < 14:
        s.p_agent.publish(busy)
        time.sleep(0.5)
    check('nothing over an agent mission', not s.selects(), s.selects())
    check('reported', s.report.get('code') == 'blocked_agent', s.report.get('code'))
    s.p_agent.publish(String(json.dumps({'state': 'READY', 'run_id': None, 'mow_program': None})))
    check('caught up after it', s.wait(lambda: s.selects(), 25, 'select'))
    check('finished', finish_ok(s))
    return s


def scenario_mux_hold():
    """gnss_watchdog holds the robot in the dock at the slot: the calendar waits,
    no sleep, and catches up after the release."""
    s = Sim()
    start(s, D(2026, 6, 10, 7, 59, 54), daily('08:00'))
    s.p_mux.publish(String('hold'))
    time.sleep(14)
    check('nothing over a mux hold', not s.selects(), s.selects())
    check('reported', s.report.get('code') == 'blocked_mux_hold', s.report.get('code'))
    check('no sleep over a mux hold', not s.sleep_req, s.sleep_req)
    s.p_mux.publish(String('nav'))
    check('caught up after it', s.wait(lambda: s.selects(), 25, 'select'))
    check('finished', finish_ok(s))
    return s


def scenario_hold_rain():
    """Rain during a mux hold on the lawn: the mission ends at once, but the way
    home waits for the release; no sleep and no resume over the hold."""
    s = Sim(master_params={'weather_sleep_duration_h': 0.003, 'idle_sleep_threshold_s': 1e7})
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('mowing path 1', s.wait(lambda: any(y == 1.0 for y, _ in s.mowed()), 60, 'path 1'))
    s.p_mux.publish(String('hold'))
    s.p_held.publish(Bool(True))
    time.sleep(3)
    n = len(s.mowed())
    docks = len(s.events('dock_program'))
    s.set(rain_now=1, past_rain=True)
    check('rain preempts during the hold',
          s.wait(lambda: s.stop_reason == 'interrupted:weather', 25, 'weather'), s.stop_reason)
    time.sleep(10)
    check('the way home waits for the release', len(s.events('dock_program')) == docks,
          s.events('dock_program'))
    check('no mowing, no sleep over the hold', len(s.mowed()) <= n + 1 and not s.sleep_req,
          (n, len(s.mowed()), s.sleep_req))
    s.p_mux.publish(String('nav'))
    s.p_held.publish(Bool(False))
    check('home after the release', s.wait(
        lambda: len(s.events('dock_program')) > docks and s.status == 'Ready', 45, 'dock'))
    s.set(rain_now=0, forecast_rain=False, past_rain=False)
    check('resumed and finished after the rain', finish_ok(s, 180))
    return s


def scenario_no_weather():
    """No weather data at all: the calendar still mows, the log says so."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    s.wait(lambda: os.path.exists(os.environ['SIM_STORE']), 10, 'store')
    s.set(no_weather=True)
    start(s, D(2026, 6, 10, 7, 59, 56), daily('08:00'))
    check('finished', finish_ok(s))
    return s


def _run_by_hand(s):
    pub = rospy.Publisher('/web_plan/program_select', String, queue_size=1, latch=True)
    s.wait(lambda: pub.get_num_connections() > 0, 15, 'planner listens')
    return lambda: pub.publish(String(daily('08:00')['program_name']))


def scenario_manual_rain():
    """rain_mode always: a Run by hand obeys the rain; "ignore this time" lets one
    run through, and only one."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7})
    s.wait(lambda: os.path.exists(os.environ['SIM_STORE']), 10, 'store')
    s.set(rain_now=1)
    time.sleep(1.5)
    go = _run_by_hand(s)
    go()
    check('refused', s.wait(lambda: s.stop_reason == 'rejected:rain', 30, 'rejected'), s.stop_reason)
    check('never undocked', not s.events('dock_program'))
    check('the report knows', s.wait(lambda: s.report.get('reject') == 'rain'
                                     and s.report.get('rain_would_refuse') == 'rain', 10, 'report'))
    once = rospy.Publisher('/scheduler/ignore_rain_once', Bool, queue_size=1)
    s.wait(lambda: once.get_num_connections() >= 2, 15, 'both listen')
    once.publish(Bool(True))
    time.sleep(0.5)
    go()
    check('ignored once: it goes', s.wait(lambda: len(s.events('dock_program')) >= 1, 40, 'undock'))
    check('finished in the rain', finish_ok(s))
    check('flag gone', s.wait(lambda: s.report.get('ignore_rain_once') is False, 10, 'cleared'))
    n = len(s.events('dock_program'))
    time.sleep(1.0)
    go()
    check('refused again', s.wait(lambda: s.stop_reason == 'rejected:rain', 30, 'rejected'), s.stop_reason)
    check('did not undock again', len(s.events('dock_program')) == n)
    return s


def scenario_manual_rain_calendar():
    """rain_mode calendar is what the robot did until now: a Run by hand does not
    care. The value reaches the mission from master_controller, not from the yaml;
    a change on the settings topic reaches it at once."""
    s = Sim(master_params={'idle_sleep_threshold_s': 1e7, 'rain_mode': 'calendar'})
    s.wait(lambda: os.path.exists(os.environ['SIM_STORE']), 10, 'store')
    check('pushed', s.wait(lambda: rospy.get_param('/node_mower_smach/weather_rain_mode') == 'calendar',
                           15, 'param'))
    s.set(rain_now=1)
    time.sleep(1.5)
    go = _run_by_hand(s)
    go()
    check('goes in the rain', s.wait(lambda: len(s.events('dock_program')) >= 1, 40, 'undock'))
    check('finished', finish_ok(s))
    pub = rospy.Publisher('/scheduler/set_settings', String, queue_size=1)
    s.wait(lambda: pub.get_num_connections() > 0, 15, 'master listens')
    pub.publish(String(json.dumps({'req': 'x', 'by': 'owner', 'values': {'rain_mode': 'always'}})))
    check('changed live', s.wait(lambda: rospy.get_param('/node_mower_smach/weather_rain_mode') == 'always',
                                 10, 'param'))
    rospy.set_param('/node_mower_smach/weather_rain_mode', 'calendar')     # a launch reload
    check('put right', s.wait(lambda: rospy.get_param('/node_mower_smach/weather_rain_mode') == 'always',
                              15, 'param'))
    return s


def run(name):
    s = None
    try:
        s = globals()['scenario_' + name]()
    finally:
        if s is None:
            import gc
            for o in gc.get_objects():
                if isinstance(o, Sim):
                    s = o
        if s is not None:
            s.close()
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(run(sys.argv[1]))
