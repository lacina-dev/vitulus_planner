"""Hardware reset of what hangs behind the power module (no rospy, testable offline).

Night of 1. 11. 2026: the mower MCU went silent at 21:35 and stayed so for 80 minutes,
through two stack restarts.  It answered at once after 66 s without power.  Nothing a
mission can do reaches a hung MCU: /mower/set_power travels over the dead link and the
MCU runs with bat_out off.  Only the 19 V rail does it, and that takes the NUC along.

So it is a sleep: the same sequence as vt_hw_reset.js and sleep_manager, W = 20 s and
T = W + 60 s, read back through /pm/power_status before the arm, the OS stopped as far
as W allows.  See master_controller._tick_hw_reset for the guards.

    wanted      the persisted run waits for the mower, and the link says silent for
                silent_s.  No run: nothing to go back to, a person's business.
    spent       one per run, max_day in 24 h, both persisted.  A reset that did not
                help is not repeated: the next one is a person's.
"""
RECOVERY = 'mower_link'
DAY_S = 86400.0


def wanted(run, link, silent_s):
    if not run or RECOVERY not in (run.get('reason'), run.get('reject')):
        return False
    link = link or {}
    return link.get('state') == 'silent' and (link.get('silent_s') or 0.0) >= silent_s


def recent(times, now):
    """A time ahead of now (a clock put back) still counts: it can only stop a reset."""
    return [t for t in times or [] if now - t < DAY_S]


def spent(run, times, now, max_day):
    """Why no more now: 'run', 'day' or None."""
    if (run or {}).get('hw_reset'):
        return 'run'
    return 'day' if len(recent(times, now)) >= max_day else None
