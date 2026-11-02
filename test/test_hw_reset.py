#!/usr/bin/env python3
"""node_planner/hw_reset.py and the sequence with its timing, on the firmware model."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from node_planner import hw_reset as hw  # noqa: E402
from node_planner import sleep_manager as sm  # noqa: E402

SILENT = {'state': 'silent', 'silent_s': 200.0}


class Wanted(unittest.TestCase):

    def test_interrupted_and_rejected(self):
        self.assertTrue(hw.wanted({'reason': 'mower_link'}, SILENT, 120.0))
        self.assertTrue(hw.wanted({'reason': 'rejected', 'reject': 'mower_link'}, SILENT, 120.0))

    def test_no_run_is_a_persons(self):
        self.assertFalse(hw.wanted(None, SILENT, 120.0))

    def test_another_wait(self):
        self.assertFalse(hw.wanted({'reason': 'weather'}, SILENT, 120.0))

    def test_link_ok_or_short(self):
        run = {'reason': 'mower_link'}
        self.assertFalse(hw.wanted(run, {'state': 'ok', 'silent_s': 0.1}, 120.0))
        self.assertFalse(hw.wanted(run, {'state': 'unknown', 'silent_s': 300.0}, 120.0))
        self.assertFalse(hw.wanted(run, {'state': 'silent', 'silent_s': 60.0}, 120.0))
        self.assertFalse(hw.wanted(run, None, 120.0))


class Spent(unittest.TestCase):
    NOW = 1000000.0

    def test_free(self):
        self.assertIsNone(hw.spent({}, [], self.NOW, 2))

    def test_one_per_run(self):
        self.assertEqual(hw.spent({'hw_reset': True}, [], self.NOW, 2), 'run')
        self.assertEqual(hw.spent({'hw_reset': 'cancelled'}, [], self.NOW, 2), 'run')

    def test_two_a_day(self):
        self.assertEqual(hw.spent({}, [self.NOW - 10, self.NOW - 20], self.NOW, 2), 'day')

    def test_older_than_a_day_forgotten(self):
        self.assertIsNone(hw.spent({}, [self.NOW - 90000, self.NOW - 10], self.NOW, 2))

    def test_clock_put_back_still_counts(self):
        self.assertEqual(hw.spent({}, [self.NOW + 500, self.NOW + 600], self.NOW, 2), 'day')


class Ttys(unittest.TestCase):
    NOW = 1000000.0

    def stat(self, ages):
        def fake(path):
            age = ages[path]
            return type('S', (), {'st_atime': self.NOW - age, 'st_mtime': self.NOW - age})()
        return fake

    def test_a_live_terminal_counts(self):
        stat = self.stat({'/dev/pts/1': 30.0})
        self.assertEqual(sm.busy_ttys(['pts/1'], 600.0, self.NOW, stat), ['pts/1'])

    def test_a_long_idle_one_does_not(self):
        stat = self.stat({'/dev/pts/0': 90000.0})
        self.assertEqual(sm.busy_ttys(['pts/0'], 600.0, self.NOW, stat), [])

    def test_output_counts_as_activity(self):
        def stat(path):
            return type('S', (), {'st_atime': self.NOW - 9000, 'st_mtime': self.NOW - 5})()
        self.assertEqual(sm.busy_ttys(['pts/2'], 600.0, self.NOW, stat), ['pts/2'])

    def test_not_knowing_counts_as_busy(self):
        def stat(path):
            raise OSError
        self.assertEqual(sm.busy_ttys(['pts/3', 'unknown'], 600.0, self.NOW, stat),
                         ['pts/3', 'unknown'])

    def test_zero_switches_it_off(self):
        self.assertEqual(sm.busy_ttys(['unknown'], 0.0, self.NOW), [])


class St(object):
    def __init__(self):
        self.sleep_status = False
        self.sleep_charged_status = False
        self.sleep_wait_standby = 180000
        self.sleep_time_interval = 900000
        self.sleep_wait_charged = 300000


class Pm(object):
    def __init__(self):
        self.st = St()
        self.sent = []

    def send_wait(self, ms):
        self.st.sleep_wait_standby = ms
        self.sent.append(('wait', ms))

    def send_time(self, ms):
        self.st.sleep_time_interval = ms
        self.sent.append(('time', ms))

    def send_offset(self, ms):
        pass

    def arm(self, mode):
        self.st.sleep_status = True
        self.sent.append(('arm', mode))

    def cancel(self):
        self.st.sleep_status = False
        self.sent.append(('cancel', None))


class Helper(object):
    def __init__(self):
        self.started = None

    def start(self):
        self.started = True

    def poll(self):
        return 0 if self.started else sm.RC_SOFT


class Sequence(unittest.TestCase):

    def setUp(self):
        self.t = [100.0]
        self.pm = Pm()
        self.helper = Helper()
        self.cfg = sm.SleepConfig()
        self.hw = sm.SleepConfig(**vars(self.cfg))
        self.hw.wait_before_standby_s = 20.0
        self.hw.final_grace_s = 5.0
        self.seq = sm.SleepSequencer(self.cfg, self.pm, lambda: (self.pm.st, 0.0), self.helper,
                                     clock=lambda: self.t[0])

    def run_s(self, seconds, block=None):
        for _ in range(int(seconds)):
            self.seq.tick(block)
            self.t[0] += 1.0

    def test_w_and_t_read_back_before_the_arm(self):
        self.seq.start(sm.MODE_TIMED, 80000, self.hw)
        self.run_s(9)
        self.assertEqual(self.pm.sent[:3], [('wait', 20000), ('time', 80000),
                                            ('arm', sm.MODE_TIMED)])
        self.assertTrue(self.helper.started)

    def test_the_os_stop_starts_inside_w(self):
        self.seq.start(sm.MODE_TIMED, 80000, self.hw)
        self.run_s(5)
        self.assertFalse(self.helper.started)
        self.run_s(5)
        self.assertTrue(self.helper.started)
        self.assertEqual(self.seq.state, sm.WAIT_CUT)
        self.assertTrue(self.seq.past_return)

    def test_block_before_the_os_stop_disarms(self):
        self.seq.start(sm.MODE_TIMED, 80000, self.hw)
        self.run_s(3)
        self.assertTrue(self.pm.st.sleep_status)
        self.run_s(8, 'off_dock')
        self.assertFalse(self.pm.st.sleep_status)
        self.assertFalse(self.helper.started)
        self.assertEqual(self.seq.result, 'off_dock')

    def test_the_next_sleep_has_its_own_timing(self):
        self.seq.start(sm.MODE_TIMED, 80000, self.hw)
        self.run_s(3)
        self.seq.state = sm.IDLE
        self.pm.st.sleep_status = False
        self.seq.start(sm.MODE_TIMED, 3600000)
        self.assertIs(self.seq.run_cfg, self.cfg)
        self.run_s(2)
        self.assertIn(('wait', 180000), self.pm.sent)


if __name__ == '__main__':
    unittest.main()
