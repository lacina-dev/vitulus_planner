"""Sleep sequencing for master_controller (no rospy, fully testable offline).

The robot sleeps with the NUC off. The power module (PM, firmware
powermodulev3_1) cuts and restores the 19 V rail; it stays powered and counts.
What the firmware does and what follows from it (main.cpp check_sleep,
set_robot_sleep, set_sleep_until_charged):

  * sleep_wait_before_standby (W) and sleep_time both count from the
    set_robot_sleep(true) command. If sleep_time <= W the PM "wakes" a NUC that
    still has 19 V, the flag clears and nothing ever powers the NUC back on
    after the OS went down. So sleep_ms is always well above W.
  * set_robot_sleep(true) clears sleep_charged_status and
    set_sleep_until_charged(true) clears sleep_status: until_charged is ONE
    command, and a timed wake and a charged wake cannot be combined.
  * until_charged has no time limit, so without contact it never ends. The only
    wake guaranteed by unpatched firmware is the timed one, hence the default
    charge mode 'timed'.
  * the OS must not go down before the PM holds the flag: a NUC in S5 with the
    PM not armed stays dead until somebody comes. Hence read-back through
    /pm/power_status before anything irreversible, and the reverse after
    'systemctl poweroff' was accepted: never cancel, the PM must cut.
  * the values live in EEPROM, so they are written only when they differ.

Sequence: PARAMS (W + sleep_time, read back) -> ARM (flag, read back) -> FINAL
(last gate check) -> WAIT_CUT (helper syncs and powers off; the PM cuts when W
expires). A block or a failure cancels the flag again (ABORT), up to the moment
poweroff is accepted. Poweroff refused (no sudoers entry): the system stays up,
stays cancellable, and the PM cuts it synced.
"""
import os
import subprocess
import time

IDLE = 'IDLE'
PARAMS = 'PARAMS'
ARM = 'ARM'
FINAL = 'FINAL'
WAIT_CUT = 'WAIT_CUT'
ABORT = 'ABORT'

MODE_TIMED = 'timed'
MODE_CHARGED = 'until_charged'

RC_SOFT = 3     # helper: poweroff refused, the system stays up


class SleepConfig(object):
    """Timing of the sequence; all times in seconds."""

    def __init__(self, **kw):
        self.wait_before_standby_s = 180.0  # W: OS shutdown has to fit in here
        self.min_off_s = 120.0              # 19 V off long enough for the NUC
        self.min_sleep_s = 900.0            # shorter sleeps are not worth it
        self.max_segment_s = 12 * 3600.0    # re-read the RTC at least this often
        self.timer_drift = 0.005            # PM clock: ceramic resonator, +-0.5 %
        self.final_grace_s = 20.0           # last chance between arm and shutdown
        self.retry_s = 3.0
        self.max_tries = 5
        self.stale_s = 5.0
        self.cut_margin_s = 60.0            # after W: the PM should have cut
        self.fail_cooldown_s = 1800.0
        self.charged_offset_s = 300.0       # until_charged only, must exceed W
        self.charge_full_h = 6.0            # 0 -> 100 %, in standby
        self.charge_min_h = 0.5
        self.charge_max_h = 6.0
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError(k)
            setattr(self, k, v)


def plan_timed_ms(duration_s, cfg):
    """Sleep time for the PM, or None when the sleep is not worth it.

    Long sleeps end in segments (the caller just decides again after a wake),
    and the PM clock tolerance always errs towards waking early.
    """
    seg = min(float(duration_s), cfg.max_segment_s)
    seg *= (1.0 - cfg.timer_drift)
    floor = max(cfg.min_sleep_s, cfg.wait_before_standby_s + cfg.min_off_s)
    if seg < floor:
        return None
    return int(seg * 1000)


def plan_charge_s(capacity, cfg):
    """Charging time estimate (+10 %) from the battery deficit, capped."""
    deficit = max(0.0, min(100.0, 100.0 - float(capacity))) / 100.0
    hours = cfg.charge_full_h * deficit * 1.1
    return 3600.0 * max(cfg.charge_min_h, min(cfg.charge_max_h, hours))


def parse_who(text):
    """Login sessions of real persons from `who` output.

    The autologin desktop session (host ':0', the seat) does not count; an
    ssh/terminal session does, idle or not.
    """
    out = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        host = line[line.rfind('(') + 1:line.rfind(')')] if '(' in line else ''
        if host.startswith(':') or parts[1].startswith(('tty7', 'seat', ':')):
            continue
        out.append(parts[1])
    return out


def active_logins(run=subprocess.check_output):
    """Terminals with a logged in person; ['unknown'] when it cannot be told."""
    try:
        return parse_who(run(['who'], timeout=2.0).decode('utf-8', 'replace'))
    except Exception:  # noqa: BLE001 - not knowing counts as occupied
        return ['unknown']


def read_boot_id(path='/proc/sys/kernel/random/boot_id'):
    try:
        with open(path) as f:
            return f.read().strip()
    except (IOError, OSError):
        return ''


class ShutdownHelper(object):
    """Runs nodes/sleep_shutdown without blocking the FSM."""

    def __init__(self, cmd):
        self.cmd = cmd
        self.proc = None

    def start(self):
        try:
            self.proc = subprocess.Popen(
                [self.cmd], stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:  # noqa: BLE001 - the PM cuts anyway, after a sync
            os.sync()
            return False

    def poll(self):
        """None while running, its exit code, RC_SOFT when it never ran."""
        return RC_SOFT if self.proc is None else self.proc.poll()


class SleepSequencer(object):
    """One sleep attempt. tick() once a second, with the current block."""

    def __init__(self, cfg, pm, status_fn, helper, log=lambda m: None,
                 clock=time.time):
        self.cfg = cfg
        self.pm = pm                # send_wait/send_time/send_offset/arm/cancel
        self.status_fn = status_fn  # -> (Power_status or None, age_s)
        self.helper = helper
        self.log = log
        self.clock = clock
        self.state = IDLE
        self.mode = None
        self.sleep_ms = 0
        self.result = None          # last attempt: 'cut', or why not
        self.armed_at = 0.0
        self._reset_try()

    # -- control --------------------------------------------------------------
    def start(self, mode, sleep_ms=0):
        self.mode = mode
        self.sleep_ms = int(sleep_ms)
        self.result = None
        self._go(PARAMS)

    @property
    def active(self):
        return self.state != IDLE

    @property
    def past_return(self):
        """Poweroff under way: cancelling would leave the NUC off with 19 V on,
        and nothing would ever power it back up."""
        return self.state == WAIT_CUT and self.helper.poll() != RC_SOFT

    def report(self):
        return {'state': self.state, 'mode': self.mode,
                'sleep_ms': self.sleep_ms, 'result': self.result}

    # -- internals ------------------------------------------------------------
    def _reset_try(self):
        self.tries = 0
        self.next_send = 0.0
        self.phase_start = self.clock()

    def _go(self, state):
        self.state = state
        self._reset_try()

    def _finish(self, result):
        self.result = result
        self.state = IDLE

    def _abort(self, reason):
        self.log('[Sleep] cancelling: %s' % reason)
        self.result = reason
        self._go(ABORT)

    def _status(self):
        st, age = self.status_fn()
        return st if st is not None and age <= self.cfg.stale_s else None

    def _due(self):
        """Time for the next send of an unconfirmed command."""
        now = self.clock()
        if now < self.next_send:
            return False
        self.next_send = now + self.cfg.retry_s
        self.tries += 1
        return True

    @property
    def _exhausted(self):
        return (self.tries >= self.cfg.max_tries
                and self.clock() >= self.next_send)

    def _flag(self, st):
        return (st.sleep_charged_status if self.mode == MODE_CHARGED
                else st.sleep_status)

    def _wait_ms(self):
        return int(self.cfg.wait_before_standby_s * 1000)

    def _offset_ms(self):
        return int(self.cfg.charged_offset_s * 1000)

    # -- states ---------------------------------------------------------------
    def tick(self, block=None):
        handler = {PARAMS: self._t_params, ARM: self._t_arm,
                   FINAL: self._t_final, WAIT_CUT: self._t_wait_cut,
                   ABORT: self._t_abort}.get(self.state)
        if handler is not None:
            handler(block)

    def _t_params(self, block):
        if block:           # nothing armed yet
            self._finish(block)
            return
        st = self._status()
        missing = []
        if st is not None:
            if st.sleep_wait_standby != self._wait_ms():
                missing.append('wait')
            if self.mode == MODE_TIMED:
                if st.sleep_time_interval != self.sleep_ms:
                    missing.append('time')
            elif st.sleep_wait_charged != self._offset_ms():
                missing.append('offset')
            if not missing:
                self._go(ARM)
                self._t_arm(block)
                return
        if self._exhausted:
            self._finish('pm_no_status' if st is None else 'pm_params')
            return
        if st is not None and self._due():
            if 'wait' in missing:
                self.pm.send_wait(self._wait_ms())
            if 'time' in missing:
                self.pm.send_time(self.sleep_ms)
            if 'offset' in missing:
                self.pm.send_offset(self._offset_ms())
        elif st is None:
            self._due()     # count the wait for the PM towards the limit

    def _t_arm(self, block):
        if block:
            self._abort(block)
            return
        st = self._status()
        if st is not None and self.tries and self._flag(st):
            self._go(FINAL)
            return
        if self._exhausted:
            self._abort('pm_no_ack')
            return
        if self._due():
            self.armed_at = self.clock()
            self.pm.arm(self.mode)

    def _t_final(self, block):
        if block:
            self._abort(block)
            return
        st = self._status()
        if st is None or not self._flag(st):
            self._abort('pm_lost')
            return
        if self.clock() - self.armed_at >= self.cfg.final_grace_s:
            self.log('[Sleep] PM armed, shutting the OS down.')
            self.helper.start()
            self._go(WAIT_CUT)

    def _t_wait_cut(self, block):
        rc = self.helper.poll()
        if rc != RC_SOFT:       # helper running, or poweroff accepted
            return
        if block:
            self._abort(block)
        elif (self.clock() - self.armed_at
              > self.cfg.wait_before_standby_s + self.cfg.cut_margin_s):
            self._abort('pm_no_cut')

    def _t_abort(self, _block):
        st = self._status()
        if st is not None and self.tries and not (
                st.sleep_status or st.sleep_charged_status):
            self.state = IDLE
            return
        if self._exhausted:
            self.log('[Sleep] PM did not confirm the cancel.')
            self.result = 'pm_cancel_failed'
            self.state = IDLE
            return
        if self._due():
            self.pm.cancel()
