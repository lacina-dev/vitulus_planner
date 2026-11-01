#!/usr/bin/env python3
"""master_controller sleep sequence against the power module firmware model.

No ROS master, virtual clock, no real helper: `poweroff` is a fake object.
Run:  python3 test/test_sleep.py
"""
import importlib.machinery
import importlib.util
import os
import shutil
import sys
import tempfile
import time
import unittest
from unittest import mock

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_PKG, 'src'))
_HOME = tempfile.mkdtemp(prefix='sleep_test_home_')
os.environ['HOME'] = _HOME

import rospy  # noqa: E402
from fake_pm import FirmwareModel, RUNNING, STANDBY  # noqa: E402
from node_planner import sleep_manager as sm  # noqa: E402

M = None
EVENTS = []
FW = None
DROP = {'n': 0}     # commands the rosserial link loses


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        data = getattr(msg, 'data', msg)
        EVENTS.append((self.name, data))
        if not self.name.startswith('/pm/') or FW is None:
            return
        if DROP['n'] > 0:
            DROP['n'] -= 1
            return
        getattr(FW, self.name[len('/pm/'):])(data)


def _load():
    global M
    rospy.init_node = lambda *a, **kw: None
    rospy.Publisher = _Pub
    rospy.Subscriber = lambda *a, **kw: None
    rospy.Service = lambda *a, **kw: None
    rospy.get_param = lambda name, default=None: default
    rospy.sleep = lambda *a, **kw: None
    for lvl in ('loginfo', 'logwarn', 'logerr'):
        setattr(rospy, lvl, lambda *a, **kw: None)
    loader = importlib.machinery.SourceFileLoader(
        'master_under_test', os.path.join(_PKG, 'nodes', 'master_controller'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    M = importlib.util.module_from_spec(spec)
    loader.exec_module(M)


class FakeHelper(object):
    """Fake shutdown: never touches the machine."""

    def __init__(self):
        self.started = 0
        self.rc = None

    def start(self):
        self.started += 1
        return True

    def poll(self):
        if not self.started:
            return sm.RC_SOFT
        return self.rc


def published(topic):
    return [d for t, d in EVENTS if t == topic]


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _load()

    def setUp(self):
        global FW
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        DROP['n'] = 0
        self.t = 1800000000.0
        self.patch = mock.patch('time.time', lambda: self.t)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        FW = FirmwareModel()
        self.fw = FW
        self.boot_file = os.path.join(_HOME, 'boot_id')
        with open(self.boot_file, 'w') as f:
            f.write('boot-1')
        self.mc = self.make_master()

    def make_master(self):
        mc = M.MasterController()
        mc.boot_id_file = self.boot_file
        mc.automation_on = True
        mc.mission_busy = False
        mc.sleep_check_logins = False
        mc.sleep_countdown_s = 0        # the countdown has its own tests
        mc.window_enabled = False
        mc.state = mc.STATE_IDLE
        mc.is_in_dock = True
        mc.programs_loaded = True
        mc.sleep_seq.clock = lambda: self.t
        self.helper = mc.sleep_seq.helper = FakeHelper()
        self.feed(mc)
        return mc

    def feed(self, mc=None):
        mc = mc or self.mc
        mc.pm_status = self.fw.status()
        mc.pm_status_time = self.t
        mc.supply_status = self.fw.supply_status
        mc.battery_capacity = self.fw.status().battery_capacity

    def run_for(self, seconds, step=1.0):
        """1 Hz ticks of master and PM; the master dies with the 19 V."""
        end = self.t + seconds
        while self.t < end:
            self.t += step
            self.fw.tick(int((self.t - 1800000000.0) * 1000))
            if self.fw.nuc_on and self.helper.rc != 0:
                self.feed()
                self.mc._step()


class Sequence(Base):
    def test_timed_arm_then_helper_then_cut_and_wake(self):
        ok = self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.assertTrue(ok)
        self.run_for(2)
        self.assertEqual(self.mc.state, self.mc.STATE_PREPARE_SLEEP)
        self.assertEqual(self.helper.started, 0)    # not before the flag + grace
        self.run_for(25)
        self.assertTrue(self.fw.sleep_status)
        self.assertEqual(self.helper.started, 1)
        self.helper.rc = 0
        self.run_for(170)
        self.assertEqual(self.fw.module_status, STANDBY)
        self.run_for(3600)
        self.assertEqual(self.fw.module_status, RUNNING)
        self.assertEqual(self.fw.log, ['cut:timed', 'wake:timed'])

    def test_helper_never_starts_before_the_pm_confirmed(self):
        DROP['n'] = 10**6
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(120)
        self.assertEqual(self.helper.started, 0)
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)
        self.assertTrue(self.fw.nuc_on)
        self.assertEqual(self.mc.sleep_block, 'pm_params')

    def test_lost_commands_are_retried(self):
        DROP['n'] = 3
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(40)
        self.assertTrue(self.fw.sleep_status)
        self.assertEqual(self.helper.started, 1)

    def test_lost_arm_cancels_and_cools_down(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(1.0)
        self.fw.set_robot_sleep = lambda on: None   # ignores arm
        self.run_for(60)
        self.assertEqual(self.helper.started, 0)
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)
        self.assertEqual(self.mc.sleep_block, 'pm_no_ack')
        self.assertFalse(self.mc._execute_sleep(duration_ms=3600 * 1000))
        self.t += self.mc.sleep_cfg.fail_cooldown_s + 1
        self.assertTrue(self.mc._execute_sleep(duration_ms=3600 * 1000))

    def test_sleep_time_always_above_w(self):
        cfg = self.mc.sleep_cfg
        for d in (0, 60, 299, 899):
            self.assertIsNone(sm.plan_timed_ms(d, cfg))
        ms = sm.plan_timed_ms(1000, cfg)
        self.assertGreater(ms, (cfg.wait_before_standby_s + cfg.min_off_s) * 1000)

    def test_too_short_stays_awake_without_touching_pm(self):
        self.assertFalse(self.mc._execute_sleep(duration_ms=60000))
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)
        self.assertEqual([e for e in EVENTS if e[0].startswith('/pm/')], [])

    def test_long_sleep_is_a_segment_and_early(self):
        ms = sm.plan_timed_ms(30 * 3600, self.mc.sleep_cfg)
        self.assertEqual(ms, int(12 * 3600 * 1000 * 0.995))
        ms = sm.plan_timed_ms(8 * 3600, self.mc.sleep_cfg)
        self.assertEqual(ms, int(8 * 3600 * 1000 * 0.995))

    def test_w_goes_to_eeprom_once(self):
        for _ in range(3):
            self.mc._execute_sleep(duration_ms=3600 * 1000)
            self.run_for(10)
            self.mc.is_inhibited = True     # cancels
            self.run_for(10)
            self.mc.is_inhibited = False
            self.mc.state = self.mc.STATE_IDLE
        self.assertEqual(len(published('/pm/set_sleep_wait_before_standby')), 1)
        self.assertEqual(self.fw.eeprom_writes, 2)  # W + one sleep_time value


class ChargeSleep(Base):
    def setUp(self):
        super().setUp()
        self.fw.capacity = 30.0

    def test_old_two_commands_are_the_bug(self):
        """fw:1661-1662: what the old master did cancels until_charged."""
        self.fw.set_sleep_until_charged(True)
        self.fw.set_robot_sleep(True)
        self.assertFalse(self.fw.sleep_charged_status)
        self.assertTrue(self.fw.sleep_status)

    def test_until_charged_mode_is_one_command(self):
        self.mc.charge_sleep_mode = sm.MODE_CHARGED
        self.assertTrue(self.mc._execute_sleep(charged=True))
        self.run_for(30)
        self.assertEqual(published('/pm/set_sleep_until_charged'), [True])
        self.assertEqual(published('/pm/set_robot_sleep'), [])
        self.assertTrue(self.fw.sleep_charged_status)
        self.assertFalse(self.fw.sleep_status)
        self.helper.rc = 0
        self.run_for(6 * 3600)
        self.assertEqual(self.fw.log[0], 'cut:charged')
        self.assertEqual(self.fw.log[-1], 'wake:charged')

    def test_w_shorter_than_charged_offset(self):
        self.mc.charge_sleep_mode = sm.MODE_CHARGED
        self.assertGreater(self.mc.sleep_cfg.charged_offset_s,
                           self.mc.sleep_cfg.wait_before_standby_s)
        self.mc._execute_sleep(charged=True)
        self.run_for(30)
        self.assertEqual(self.fw.offset_ms, 300000)

    def test_default_is_timed_and_ends_without_contact(self):
        self.assertTrue(self.mc._execute_sleep(charged=True))
        self.run_for(30)
        self.assertEqual(published('/pm/set_sleep_until_charged'), [])
        self.assertTrue(self.fw.sleep_status)
        self.helper.rc = 0
        self.run_for(200)
        self.assertEqual(self.fw.module_status, STANDBY)
        self.fw.contact = False     # lost in the sleep
        self.run_for(6 * 3600 + 60)
        self.assertEqual(self.fw.module_status, RUNNING)

    def test_until_charged_without_contact_never_ends(self):
        """Why 'timed' is the default with this firmware."""
        self.fw.wait_ms = 1000
        self.fw.set_sleep_until_charged(True)
        self.fw.contact = False
        for h in range(1, 100):
            self.fw.tick(h * 3600 * 1000)
        self.assertEqual(self.fw.module_status, STANDBY)

    def test_duration_scales_and_is_capped(self):
        cfg = self.mc.sleep_cfg
        self.assertEqual(sm.plan_charge_s(0, cfg), 6 * 3600.0)
        self.assertEqual(sm.plan_charge_s(99, cfg), 0.5 * 3600.0)
        self.assertAlmostEqual(sm.plan_charge_s(50, cfg), 3.3 * 3600.0)

    def test_capped_before_next_program(self):
        with mock.patch.object(self.mc, '_next_program_s', lambda: 2 * 3600.0):
            self.mc._execute_sleep(charged=True)
        self.assertLessEqual(self.mc.sleep_seq.sleep_ms,
                             (2 * 3600 - self.mc.wake_lead_s) * 1000)

    def test_no_contact_refused(self):
        self.fw.contact = False
        self.fw.tick(1000)
        self.feed()
        self.assertFalse(self.mc._execute_sleep(charged=True))
        self.assertEqual(self.mc.sleep_block, 'no_charging')
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)

    def test_already_charged_refused(self):
        self.fw.charger_status = 'CHARGED'
        self.feed()
        self.assertFalse(self.mc._execute_sleep(charged=True))
        self.assertEqual(self.mc.sleep_block, 'charged')

    def _charge_cycle(self, capacity_after):
        self.assertTrue(self.mc._execute_sleep(charged=True))
        self.fw.capacity = capacity_after
        self.feed()
        self.mc._rate_charge_sleep(self.mc.pm_status)
        self.mc.sleep_seq.state = sm.IDLE
        self.mc.state = self.mc.STATE_IDLE

    def test_no_gain_cap(self):
        self._charge_cycle(30.0)
        self._charge_cycle(30.0)
        self.assertFalse(self.mc._execute_sleep(charged=True))
        self.assertEqual(self.mc.sleep_block, 'charge_no_gain')

    def test_charge_mission_charge_mission_is_not_no_gain(self):
        for _ in range(5):
            self.fw.capacity = 25.0     # the mission between the sleeps
            self.feed()
            self._charge_cycle(90.0)
        self.assertEqual(self.mc.persistent['charge_sleep']['no_gain'], 0)


class Gates(Base):
    def assert_cancelled(self, block):
        self.assertEqual(self.helper.started, 0)
        self.assertFalse(self.fw.sleep_status)
        self.assertTrue(self.fw.nuc_on)
        self.assertEqual(self.mc.sleep_block, block)

    def test_refusals_touch_nothing(self):
        mc = self.mc
        for attr, val, block in (('automation_on', False, 'disabled'),
                                 ('is_inhibited', True, 'hold'),
                                 ('is_in_dock', False, 'off_dock')):
            old = getattr(mc, attr)
            setattr(mc, attr, val)
            self.assertFalse(mc._execute_sleep(duration_ms=3600 * 1000))
            self.assertEqual(mc.sleep_block, block)
            setattr(mc, attr, old)
        self.assertEqual([e for e in EVENTS if e[0].startswith('/pm/')], [])

    def test_switch_off_leads_to_disabled(self):
        self.mc.automation_on = False
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.assertEqual(self.mc.state, self.mc.STATE_DISABLED)

    def test_switch_off_after_arm_cancels(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.assertTrue(self.fw.sleep_status)
        self.mc.automation_on = False
        self.run_for(20)
        self.assert_cancelled('disabled')
        self.assertEqual(self.mc.state, self.mc.STATE_DISABLED)
        self.run_for(300)
        self.assertTrue(self.fw.nuc_on)

    def test_switch_callback_during_sleep_keeps_the_sequence(self):
        """The real switch path: from a callback, between two ticks."""
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.mc.set_automation(False)
        self.assertEqual(self.mc.state, self.mc.STATE_PREPARE_SLEEP)
        self.run_for(20)
        self.assert_cancelled('disabled')
        self.assertEqual(self.mc.state, self.mc.STATE_DISABLED)

    def test_person_logs_in_after_arm(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.mc.sleep_check_logins = True
        self.mc._logins_cache = (self.t + 100, ['pts/3'])
        self.run_for(20)
        self.assert_cancelled('login:pts/3')
        self.assertEqual(self.mc.state, self.mc.STATE_IDLE)

    def test_open_ui_keeps_it_awake(self):
        self.mc.ui_clients = 2
        self.assertFalse(self.mc._execute_sleep(duration_ms=3600 * 1000))
        self.assertEqual(self.mc.sleep_block, 'ui:2')
        self.mc.ui_clients = 0
        self.assertTrue(self.mc._execute_sleep(duration_ms=3600 * 1000))

    def test_stopped_robot_waits_for_a_person(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.mc.smach_status = 'Stopped'
        self.run_for(20)
        self.assert_cancelled('busy:Stopped')

    def test_mission_blocks_after_arm(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.mc._mission_blocked = lambda: 'agent'
        self.run_for(20)
        self.assert_cancelled('busy:agent')

    def test_leaving_dock_after_arm(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(8)
        self.mc.is_in_dock = False
        self.run_for(20)
        self.assert_cancelled('off_dock')

    def test_no_cancel_after_poweroff_accepted(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(25)
        self.helper.rc = 0
        self.mc.automation_on = False
        del EVENTS[:]
        self.mc._step()
        self.assertEqual(published('/pm/set_robot_sleep'), [])
        self.assertTrue(self.fw.sleep_status)

    def test_poweroff_refused_pm_cuts_a_running_system(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(25)
        self.helper.rc = sm.RC_SOFT
        self.run_for(170)
        self.assertEqual(self.fw.module_status, STANDBY)

    def test_poweroff_refused_is_still_cancellable(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(25)
        self.helper.rc = sm.RC_SOFT
        self.mc.is_inhibited = True
        self.run_for(10)
        self.assertFalse(self.fw.sleep_status)
        self.assertTrue(self.fw.nuc_on)

    def test_pm_never_cuts_is_bounded(self):
        self.mc._execute_sleep(duration_ms=3600 * 1000)
        self.run_for(25)
        self.helper.rc = sm.RC_SOFT
        self.fw.wait_ms = 10**9     # keeps armed, does not cut
        self.run_for(300)
        self.assertNotEqual(self.mc.state, self.mc.STATE_PREPARE_SLEEP)
        self.assertEqual(self.mc.sleep_block, 'pm_no_cut')

    def test_parse_who(self):
        text = ("vitulus  :0           2026-10-30 22:21 (:0)\n"
                "vitulus  pts/0        2026-10-30 22:54 (192.168.0.7)\n"
                "vitulus  pts/4        2026-10-31 12:32 (192.168.0.7)\n")
        self.assertEqual(sm.parse_who(text), ['pts/0', 'pts/4'])
        self.assertEqual(sm.parse_who(''), [])

    def test_who_failure_counts_as_occupied(self):
        def boom(*a, **kw):
            raise OSError('no who')
        self.assertEqual(sm.active_logins(run=boom), ['unknown'])


class OffDock(Base):
    def setUp(self):
        super().setUp()
        self.mc.is_in_dock = False
        self.mc.state = self.mc.STATE_WAIT_OFF_DOCK
        self.mc.off_dock_since = self.t
        self.mc.dock_program = 'DOCKPROGRAM'

    def test_never_sleeps_on_the_grass(self):
        self.mc._pending_recovery_reason = None
        self.run_for(self.mc.off_dock_wait_s + 600)
        self.assertEqual([e for e in EVENTS if e[0].startswith('/pm/')], [])
        self.assertTrue(self.fw.nuc_on)
        self.assertEqual(self.mc.state, self.mc.STATE_WAIT_OFF_DOCK)

    def test_owner_stop_is_not_sent_home(self):
        self.mc._pending_recovery_reason = None
        self.run_for(self.mc.off_dock_wait_s + 5)
        self.assertEqual(published('/dock_smach/start_docking'), [])

    def test_automatic_recovery_goes_home_once(self):
        self.mc._pending_recovery_reason = 'weather'
        self.run_for(self.mc.off_dock_wait_s + 600)
        self.assertEqual(published('/dock_smach/start_docking'), ['DOCKPROGRAM'])

    def test_hold_and_busy_do_not_go_home(self):
        self.mc._pending_recovery_reason = 'weather'
        self.mc.is_inhibited = True
        self.mc.state = self.mc.STATE_WAIT_OFF_DOCK
        self.mc._handle_wait_off_dock()
        self.mc.off_dock_since = self.t - self.mc.off_dock_wait_s - 1
        self.mc._handle_wait_off_dock()
        self.assertEqual(published('/dock_smach/start_docking'), [])


class AfterStart(Base):
    def _intent(self, boot='boot-1'):
        self.mc.persistent['sleep_intent'] = {
            'mode': 'timed', 'boot_id': boot, 'started': self.t,
            'planned_wake': self.t + 100}
        self.mc._save_state()

    def test_restart_mid_sequence_cancels_the_armed_pm(self):
        self._intent()
        self.fw.set_robot_sleep(True)
        mc = self.make_master()     # same boot: a node restart
        self.mc = mc
        self.run_for(5)
        self.assertFalse(self.fw.sleep_status)
        self.assertTrue(mc._sleep_start_checked)
        self.assertNotIn('sleep_intent', mc.persistent)
        self.assertIsNone(mc.last_wake)

    def test_ui_sleep_is_left_alone(self):
        self.fw.set_robot_sleep(True)
        mc = self.make_master()
        self.mc = mc
        mc._step()
        self.assertTrue(self.fw.sleep_status)
        self.assertEqual(published('/pm/set_robot_sleep'), [])

    def test_wake_in_a_new_boot_reports_lateness(self):
        self._intent()
        self.t += 700
        with open(self.boot_file, 'w') as f:
            f.write('boot-2')
        self.mc = self.make_master()
        self.mc._step()
        self.assertAlmostEqual(self.mc.last_wake['late_s'], 600.0)
        self.assertNotIn('sleep_intent', self.mc.persistent)

    def test_sleep_confirmed_is_gone(self):
        with open(os.path.join(_PKG, 'nodes', 'master_controller')) as f:
            self.assertNotIn('sleep_confirmed', f.read())

    def test_no_wait_for_gps_anywhere_before_undock(self):
        with open(os.path.join(_PKG, 'src', 'node_planner',
                               'sleep_manager.py')) as f:
            text = f.read().lower()
        self.assertNotIn('rtk', text)


if __name__ == '__main__':
    unittest.main()
