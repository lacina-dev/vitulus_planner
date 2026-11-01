#!/usr/bin/env python3
"""Sleep/wake with the real master_controller process on a private roscore.

fake_pm.FirmwareModel runs as /pm; "19 V off" kills the master process and
"19 V on" starts it again with a new boot id, like a real NUC. The shutdown
helper is a stub that only writes a file. Scaled times (W = 4 s).

  python3 test/sim_sleep.py          the scenarios below against nodes/master_controller
  python3 test/sim_sleep.py --old    same against HEAD's master (git show):
                                     until_charged must be seen failing
Port 11860 + pid % 60, ROS_HOME and HOME in a temp dir. Touches nothing live:
it starts only its own processes and kills only those.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
PORT = 11860 + os.getpid() % 60
TMP = tempfile.mkdtemp(prefix='sim_sleep_')
ENV = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
           ROS_IP='127.0.0.1', ROS_HOME=os.path.join(TMP, 'ros'), HOME=TMP)
ENV.pop('ROS_HOSTNAME', None)
os.environ.update({k: ENV[k] for k in ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOME')})

import rospy  # noqa: E402
from std_msgs.msg import Bool, Int8, String, UInt64  # noqa: E402
from vitulus_msgs.msg import (DockProgram, PlannerProgram,  # noqa: E402
                              PlannerProgramList, Power_status)
from fake_pm import FirmwareModel, RUNNING  # noqa: E402

PARAMS = {
    'automation_enabled': True, 'startup_delay_s': 0.2,
    'boot_grace_period_s': 0.0, 'check_interval_s': 0.5,
    'allowed_window_enabled': False, 'sleep_wait_before_standby_s': 4.0,
    'sleep_min_off_s': 1.0, 'sleep_min_s': 3.0, 'sleep_final_grace_s': 1.0,
    'sleep_retry_s': 1.0, 'sleep_cut_margin_s': 3.0,
    'sleep_check_logins': False, 'sleep_countdown_s': 0.0,
    'sleep_charged_offset_s': 5.0,
    'wake_before_program_s': 0.0, 'boot_time_buffer_s': 0.0,
    'idle_sleep_threshold_s': 1.0,
    'boot_id_file': os.path.join(TMP, 'boot_id'),
    'sleep_shutdown_cmd': os.path.join(TMP, 'stub_shutdown'),
}


class World(object):
    """/pm + the topics of the rest of the robot, and the NUC's power."""

    def __init__(self, program):
        self.fw = FirmwareModel(now_ms=0)
        self.t0 = time.time()
        self.program = program
        self.proc = None
        self.boots = 0
        self.topics = []
        ns = '/pm/'
        for name, kind in (('set_robot_sleep', Bool),
                           ('set_sleep_until_charged', Bool),
                           ('set_sleep_time', UInt64),
                           ('set_sleep_wait_before_standby', UInt64),
                           ('set_sleep_wait_charged_offset', UInt64)):
            rospy.Subscriber(ns + name, kind, self._cmd, name)
        self.pm_pub = rospy.Publisher(ns + 'power_status', Power_status,
                                      queue_size=1)
        self.dock = rospy.Publisher('/dock_manager/is_in_dock_confirmed', Bool,
                                    queue_size=1, latch=True)
        self.plist = rospy.Publisher('/web_plan/program_list', PlannerProgramList,
                                     queue_size=1, latch=True)
        self.smach = rospy.Publisher('/mower_smach/status', String, queue_size=1,
                                     latch=True)
        self.busy = rospy.Publisher('/mower_smach/busy', Bool, queue_size=1,
                                    latch=True)
        self.dock_status = rospy.Publisher('/dock_smach/dock_status',
                                           Int8,
                                           queue_size=1, latch=True)
        self.dock_prog = rospy.Publisher('/dock_manager/dock_program', DockProgram,
                                         queue_size=1, latch=True)
        self.status = ''
        rospy.Subscriber('/scheduler/status', String, self._status)
        rospy.sleep(0.5)
        self.dock.publish(Bool(True))
        self.smach.publish(String('Ready'))
        self.busy.publish(Bool(False))
        self.dock_status.publish(0)
        lst = PlannerProgramList()
        lst.program_list = [PlannerProgram(name='MowAll')]
        self.plist.publish(lst)

    def _status(self, msg):
        self.status = msg.data

    def _cmd(self, msg, name):
        self.topics.append((round(time.time() - self.t0, 1), name, msg.data))
        if self.fw.module_status == RUNNING:    # rosserial needs the NUC
            getattr(self.fw, name)(msg.data)

    def start_master(self):
        self.boots += 1
        with open(PARAMS['boot_id_file'], 'w') as f:
            f.write('boot-%d' % self.boots)
        log = open(os.path.join(TMP, 'master-%d.log' % self.boots), 'w')
        self.proc = subprocess.Popen(
            [sys.executable, self.program], env=ENV, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)

    def stop_master(self):
        if self.proc is not None:
            os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait()
            self.proc = None

    def step(self):
        was_on = self.fw.nuc_on
        self.fw.tick(int((time.time() - self.t0) * 1000))
        if was_on and not self.fw.nuc_on:
            self.stop_master()
        elif self.fw.nuc_on and not was_on:
            self.start_master()
        if self.fw.nuc_on:
            s, m = self.fw.status(), Power_status()
            for f in ('module_status', 'supply_status', 'charger_status',
                      'battery_capacity', 'sleep_status', 'sleep_charged_status',
                      'sleep_wait_standby', 'sleep_time_interval',
                      'sleep_wait_charged'):
                setattr(m, f, getattr(s, f))
            self.pm_pub.publish(m)

    def run(self, seconds, until=None):
        end = time.time() + seconds
        while time.time() < end and not rospy.is_shutdown():
            self.step()
            if until is not None and until():
                return True
            time.sleep(0.1)
        return False


def write_state(extra):
    d = os.path.join(TMP, '.vitulus')
    os.makedirs(d, exist_ok=True)
    state = {'calendar': [], 'last_run_by_event': {}, 'inhibit': False,
             'interrupted_event_id': None, 'interrupted_reason': None,
             'automation_enabled': True}
    state.update(extra)
    with open(os.path.join(d, 'scheduler_state.json'), 'w') as f:
        json.dump(state, f)


def read_state():
    with open(os.path.join(TMP, '.vitulus', 'scheduler_state.json')) as f:
        return json.load(f)


def far_event():
    """An event hours away, so IDLE wants a long sleep."""
    return {'id': 'ev1', 'program_name': 'MowAll', 'enabled': True,
            'recurrence': 'daily', 'start_time':
            time.strftime('%H:%M', time.localtime(time.time() + 6 * 3600))}


def set_params(extra):
    params = dict(PARAMS, **extra)
    for k, v in params.items():
        rospy.set_param('/master_controller/' + k, v)


def scenario_timed(program):
    """Idle sleep, cut, wake in a new boot, sleep again (a segment)."""
    write_state({'calendar': [far_event()]})
    set_params({'sleep_max_segment_h': 8.0 / 3600})
    w = World(program)
    w.fw.supply_status = 'ONLINE'
    w.start_master()
    ok = w.run(60, lambda: w.fw.log.count('wake:timed') >= 2)
    w.stop_master()
    res = {'cycles': w.fw.log, 'boots': w.boots}
    res['ok'] = (ok and w.fw.log[:2] == ['cut:timed', 'wake:timed']
                 and os.path.exists(os.path.join(TMP, 'shutdown_ran')))
    return res


def scenario_charge(program, mode):
    """Battery recovery. Passes when the robot sleeps with the 19 V off and
    gets woken only by the PM's own condition."""
    event = far_event()
    write_state({'calendar': [event], 'interrupted_event_id': 'ev1',
                 'interrupted_reason': 'battery'})
    set_params({'charge_sleep_mode': mode, 'sleep_max_segment_h': 12.0,
                'battery_recovery_min_capacity': 40,
                'charge_full_h': 15.0 / 3600, 'charge_sleep_min_h': 5.0 / 3600,
                'charge_sleep_max_h': 15.0 / 3600})
    w = World(program)
    w.fw.capacity = 20.0
    w.fw.charge_pct_per_ms = 0.0
    w.start_master()
    armed_charged = []

    def watch():
        if w.fw.sleep_charged_status:
            armed_charged.append(True)
        if not w.fw.nuc_on:
            w.fw.charge_pct_per_ms = 100.0 / 12000
        return any(x.startswith('wake:') for x in w.fw.log)

    woke = w.run(50, watch)
    w.stop_master()
    res = {'log': w.fw.log, 'mode': mode, 'armed_charged': bool(armed_charged),
           'commands': [t for t in w.topics if 'sleep' in t[1]]}
    res['ok'] = (woke and w.fw.log[0].startswith('cut:')
                 and (mode != 'until_charged'
                      or (bool(armed_charged) and 'wake:charged' in w.fw.log)))
    return res


def scenario_no_contact(program):
    """Default mode: contact lost in the sleep, the timer still wakes it."""
    write_state({'calendar': [far_event()], 'interrupted_event_id': 'ev1',
                 'interrupted_reason': 'battery'})
    set_params({'charge_sleep_mode': 'timed', 'battery_recovery_min_capacity': 40,
                'charge_full_h': 12.0 / 3600, 'charge_sleep_min_h': 5.0 / 3600,
                'charge_sleep_max_h': 12.0 / 3600})
    w = World(program)
    w.fw.capacity = 20.0
    w.start_master()
    cut = w.run(30, lambda: 'cut:timed' in w.fw.log)
    w.fw.contact = False
    woke = w.run(30, lambda: 'wake:timed' in w.fw.log)
    w.stop_master()
    return {'log': w.fw.log, 'ok': cut and woke}


def main():
    old = '--old' in sys.argv
    program = os.path.join(PKG, 'nodes', 'master_controller')
    if old:
        src = subprocess.check_output(['git', '-C', PKG, 'show',
                                       'HEAD:nodes/master_controller'])
        program = os.path.join(TMP, 'master_old')
        with open(program, 'wb') as f:
            f.write(src)
    stub = PARAMS['sleep_shutdown_cmd']
    with open(stub, 'w') as f:
        f.write('#!/bin/sh\ntouch %s/shutdown_ran\nexit 0\n' % TMP)
    os.chmod(stub, 0o755)

    core = subprocess.Popen(['roscore', '-p', str(PORT)], env=ENV,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    failed = []
    try:
        time.sleep(3)
        rospy.init_node('sim_world', anonymous=True, disable_signals=True)
        cases = [('until_charged', lambda: scenario_charge(program, 'until_charged'))]
        if not old:
            cases += [('timed', lambda: scenario_timed(program)),
                      ('charge_timed', lambda: scenario_charge(program, 'timed')),
                      ('no_contact', lambda: scenario_no_contact(program))]
        for name, fn in cases:
            res = fn()
            print(name, json.dumps(res, default=str))
            sys.stdout.flush()
            if not res['ok']:
                failed.append(name)
    finally:
        os.killpg(core.pid, signal.SIGINT)
        core.wait()
        if '--keep' in sys.argv:
            print('kept', TMP)
        else:
            shutil.rmtree(TMP, ignore_errors=True)
    if old:
        print('OLD master: until_charged scenario %s'
              % ('FAILED as expected' if failed else 'PASSED?!'))
        return 0 if failed else 1
    print('FAILED: %s' % failed if failed else 'ALL OK')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
