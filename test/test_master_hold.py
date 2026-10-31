#!/usr/bin/env python3
"""master_controller: Hold (/scheduler/inhibit) during a running program.

No ROS master, no robot: rospy publishers / subscribers / params / sleep are
stubbed and the node keeps its state file in a temporary $HOME.

Covers
  * Hold while MOWING: the program is still followed to its end; a program
    that SUCCEEDED is not resumed after Hold is released (before: master
    left MOWING, the event stayed "interrupted" and release resumed it)
  * Hold + recovery reason (battery): no sleep under Hold, the event and the
    reason stay, release resumes it
  * without Hold the behaviour is unchanged (battery -> sleep until charged)
  * status while the mission waits for Reset after STOP
  * cmd_vel_mux hold (gnss_watchdog, also escalated): no start, no resume and no
    sleep over it; the slot waits; a robot sent home is still sent (dock_smach
    and the mission wait the hold out themselves)

Run:  python3 test/test_master_hold.py
"""
import importlib.machinery
import importlib.util
import os
import shutil
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_HOME = tempfile.mkdtemp(prefix='master_test_home_')
os.environ['HOME'] = _HOME

try:
    import rospy
    from std_msgs.msg import Bool, String
    _SKIP = None
except Exception as _e:             # pragma: no cover
    _SKIP = 'ROS python libs not importable: {}'.format(_e)

M = None
EVENTS = []


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        EVENTS.append((self.name, msg))


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


def published(topic):
    return [m.data for t, m in EVENTS if t == topic]


@unittest.skipIf(_SKIP is not None, _SKIP)
class MasterHoldTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _load()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_HOME, ignore_errors=True)

    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        del EVENTS[:]
        mc = M.MasterController()
        assert mc.state_file.startswith(_HOME), mc.state_file
        mc.window_enabled = False
        mc.sleep_check_logins = False
        mc.boot_grace_period_s = 0
        mc.check_interval_s = 0
        mc.programs_loaded = True
        mc.program_list = [type('P', (), {'name': 'MowAll'})()]
        mc.is_in_dock = True
        mc.battery_capacity = 90
        mc.pm_status = type('S', (), {'charger_status': 'CHARGING',
                                      'supply_status': 'ONLINE',
                                      'battery_capacity': 30})()
        mc.calendar = [{'id': 'ev1', 'program_name': 'MowAll', 'enabled': True,
                        'recurrence': 'daily', 'start_time': '08:00'}]
        mc.automation_on = True
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.state = mc.STATE_IDLE
        self.mc = mc

    def _start_scheduled(self):
        """The calendar started MowAll (what _handle_idle does when due)."""
        mc = self.mc
        mc.persistent['interrupted_event_id'] = 'ev1'
        mc.persistent['last_run_by_event']['ev1'] = '2099-01-01'
        mc._start_program('MowAll', mc.program_select_pub)
        mc._mowing_grace_until = 0
        mc.smach_status = 'Mowing started'
        mc.active_program_name = 'MowAll'
        mc._step()
        self.assertEqual(mc.state, mc.STATE_MOWING)

    def _end(self, reason):
        mc = self.mc
        mc.smach_status = 'Ready'
        mc.stop_reason = reason
        mc.active_program_name = ' '

    def _hold(self, on):
        self.mc.inhibit_callback(Bool(on))

    def test_hold_while_mowing_follows_the_program_to_its_end(self):
        mc = self.mc
        self._start_scheduled()
        self._hold(True)
        mc._step()
        self.assertEqual(mc.state, mc.STATE_MOWING)      # still followed
        self._end('succeeded')
        mc._step()                                         # program end seen
        mc._step()
        self.assertEqual(mc.state, mc.STATE_INHIBITED)
        self.assertIsNone(mc.persistent['interrupted_event_id'])
        self._hold(False)
        del EVENTS[:]
        mc._step()
        mc._step()
        self.assertEqual(published('/web_plan/program_select_resume'), [])
        self.assertEqual(published('/web_plan/program_select'), [])
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)  # idle sleep, no start

    def test_hold_skips_the_recovery_sleep_and_resumes_after_release(self):
        mc = self.mc
        self._start_scheduled()
        self._hold(True)
        self._end('interrupted:battery')
        del EVENTS[:]
        mc._step()
        self.assertEqual(published('/pm/set_robot_sleep'), [])
        self.assertEqual(published('/pm/set_sleep_until_charged'), [])
        mc._step()
        self.assertEqual(mc.state, mc.STATE_INHIBITED)
        self.assertEqual(mc.persistent['interrupted_event_id'], 'ev1')
        self.assertEqual(mc.persistent['interrupted_reason'], 'battery')
        self._hold(False)
        mc.battery_capacity = 95                          # charged meanwhile
        mc._step()
        mc._step()
        self.assertEqual(published('/web_plan/program_select_resume'), ['MowAll'])

    def test_without_hold_battery_still_sleeps_until_charged(self):
        mc = self.mc
        self._start_scheduled()
        self._end('interrupted:battery')
        del EVENTS[:]
        mc._step()
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)
        self.assertEqual(mc.sleep_seq.mode, 'timed')    # test_sleep.py has the rest

    # -- cmd_vel_mux hold ---------------------------------------------------
    def _idle_with_slot(self):
        """Run open and due, as _handle_idle sees a slot or a resume."""
        mc = self.mc
        mc.persistent['run'] = mc._new_run(mc.calendar[0], mc._now())
        mc.smach_status = 'Ready'
        mc.mission_busy = False
        mc.dock_status = 0
        del EVENTS[:]

    def test_mux_hold_stops_a_due_start(self):
        mc = self.mc
        self._idle_with_slot()
        mc.mux_active = 'hold'
        mc._step()
        self.assertEqual(published('/web_plan/program_select'), [])
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.assertEqual(mc._code, 'blocked_mux_hold')
        self.assertIsNotNone(mc.persistent['run'])         # the run waits
        mc.mux_active = 'nav'
        mc._step()
        self.assertEqual(published('/web_plan/program_select'), ['MowAll'])

    def test_mux_hold_stops_a_resume(self):
        mc = self.mc
        self._idle_with_slot()
        mc.persistent['run']['started'] = True
        mc.mux_active = 'hold'
        mc._step()
        self.assertEqual(published('/web_plan/program_select_resume'), [])
        mc.mux_active = 'nav'
        mc._step()
        self.assertEqual(published('/web_plan/program_select_resume'), ['MowAll'])

    def test_mux_hold_refuses_sleep(self):
        mc = self.mc
        self._idle_with_slot()
        mc.mux_active = 'hold'
        self.assertIs(mc._execute_sleep(duration_ms=3600000), False)
        self.assertEqual(mc.sleep_block, 'busy:mux_hold')
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.assertEqual(published('/pm/set_robot_sleep'), [])

    def test_mux_hold_cancels_a_prepared_sleep(self):
        mc = self.mc
        self._idle_with_slot()
        mc.persistent['run'] = None
        mc.mux_active = 'nav'
        self.assertIs(mc._execute_sleep(duration_ms=3600000), True)
        self.assertEqual(mc.state, mc.STATE_PREPARE_SLEEP)
        mc.mux_active = 'hold'
        for _ in range(3):
            mc._step()
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.assertEqual(mc.sleep_block, 'busy:mux_hold')

    def test_mux_hold_still_sends_an_outside_robot_home(self):
        mc = self.mc
        self._idle_with_slot()
        mc.mux_active = 'hold'
        mc.dock_program = String('dock')
        mc._go_home_or_report()
        self.assertEqual(len(published('/dock_smach/start_docking')), 1)

    def test_status_while_stopped_waits_for_reset(self):
        mc = self.mc
        self._start_scheduled()
        mc.smach_status = 'Stopped'
        mc.stop_reason = 'stopped:user_stop'
        del EVENTS[:]
        mc._step()
        self.assertEqual(published('/scheduler/status'), ['Mowing (Stopped - press Reset)'])
        self._end('stopped:user_stop')                    # Reset pressed
        mc._step()
        self.assertEqual(mc.state, mc.STATE_IDLE)
        self.assertIsNone(mc.persistent['interrupted_event_id'])


if __name__ == '__main__':
    unittest.main()
