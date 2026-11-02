# Autonomy contract (2026-11-01)

Binding interface for the Autonomy page. Goal: one place where a person sees what the robot does on its
own and why, and controls all of it in plain language. Facts below were checked in code; line numbers are
indicative.

Owner's decisions: own menu item "Autonomie"; rain is checked for manual runs too, with "ignore this time"
at Run; the agent reads and proposes, and may Hold, but does not change settings. The robot wakes reliably
from a timed sleep through the power module (owner's experience); clean-shutdown sudoers is not installed,
the PM cut stays as it is today, do not change it.

## 1. Settings store

`~/.vitulus/autonomy.json`, owned by `master_controller`, atomic write, `version` field. A missing file or
key means the yaml default. Nothing is ever lost on a bad file: keep it aside, use defaults, report an issue.

| Topic | Dir | Payload |
|---|---|---|
| `/scheduler/settings` | latched | `{"version","values":{...},"defaults":{...},"schema":{key:{type,min,max,unit,group,advanced,locked}},"changed_at","by"}` |
| `/scheduler/set_settings` | in | `{"req","by":"owner","values":{...}}`, partial; `"reset":["key"]` |
| `/scheduler/settings_result` | latched | `{"req","ok","rejected":{key:reason}}` |

Applied at once, no node restart, each setting read at the moment of use. Master pushes the mission's values
as rosparams on `node_mower_smach` (they are read per check and are not persisted, so master pushes on start
and on change) and weather values to `weather_alert` where it reads them live; a setting that can't be live
states so in `schema` (`needs_restart`).

Keys, grouped. Defaults equal today's yaml; say so in tests.

- `plan`: `window_enabled, window_start, window_end, window_min_left_min` (replace `set_window`, keep the topic
  working), `catchup_max_h`, `skip_next` (one-shot), `no_mow_until` (date, "dnes nesekat").
- `rain`: `rain_mode`: `always | calendar | ignore`, default `always`. Replaces `weather_rain_bypass` and the
  hidden `scheduled` logic. `rain_after_min` (hard-coded 60 min today, states.py:355-361 becomes a param),
  `rain_forecast_min`, `rain_retry_h` (`weather_sleep_duration_h`), `rain_return_home` (true: leave and go home
  on rain during a run), `rain_stale_ok` (start when data are old; default true as today).
  One-shot run flag `/scheduler/ignore_rain_once` set by the Run dialog, cleared at run end.
- `battery`: `battery_start_pct` (one value, master pushes it to `battery_min_start_pct` and
  `battery_recovery_min_capacity`), `battery_return_pct`, `battery_resume_pct`, `charge_sleep_max_h`.
- `sleep`: `sleep_enabled`, `sleep_ignore_ssh`, `sleep_ignore_ui`, `idle_sleep_threshold_min`,
  `wake_before_min` (`wake_before_program_s` + `boot_time_buffer_s`), `sleep_after_run`, `stay_awake_until`
  (one-shot time).
- `recovery`: `temp_wait_h`, `temp_spike_wait_min`, `error_retry_max`, `resume_max`, `run_max_age_h`,
  `off_dock_wait_min`.

Locked, shown but not settable: critical battery, firmware and safety temperature limits, tilt, STOP, blade
rules. Advanced timing stays under `advanced: true`.

## 2. Report additions

`/scheduler/report` keeps all fields and adds:

- `sentence`: machine form `{"code","vars"}` of what it does now and next. UI writes the sentence, en + cs.
- `checks`: list `{"key","state":"ok|wait|block|off|unknown","code","vars"}` for "could it start now":
  automation, hold, dock, mission, battery, rain, gnss, temperature, window, programs, map. Computed from the
  same code that decides. No second implementation that can disagree.
- `sleep.checks`: same for "why it doesn't sleep"; `block` always current, cleared when gone (today stale).
- `run.reject` exposed. `waiting_temp_spike` handled.
- Hardware reset (`auto_hw_reset`, off by default, on per robot; `node_planner/hw_reset.py`): the run waits for the mower (`interrupted:mower_link` or `rejected:mower_link`) and /mower/link says silent for `hw_reset_silent_s`. From IDLE past the boot grace, in the dock, supply ONLINE, nothing armed, no mission, manual or agent use, no hold or stay-awake. Announced at least 60 s on /scheduler/sleep_countdown (`reason: hw_reset`), cancel on /scheduler/sleep_cancel. Then the sleep sequence with W 20 s, T 80 s; the OS stop starts 5 s after the arm. The run resumes after the boot from its checkpoint; W and T go back. One per run, 2 in 24 h (`hw_resets` in the state file), then `hw_reset_spent`. A person not ignored by `sleep_ignore_*`, or an ignored terminal used in the last `hw_reset_tty_active_s`: `hw_reset_person`, no cut while that holds. Cancel counts for the run, not for the 24 h. /scheduler/notify: code `mower_link` (information), `hw_reset_spent` (needs a person).
- `checks`: key `mower` from /mower/link (`ok`, `block`/`silent`, `wait`/`waiting_mower_link`, `unknown`/`not_checked`); `temperature` reads the board when it talks. `interrupted:mower_link` waits `mower_link_wait_min` (rosparam, awake), issue code `mower_link`, notify code `run_failed`.
- `timeline`: next 7 days of planned starts with skips, and expected sleep windows.
- `journal`: last 100 entries `{"t","kind","code","vars","program"}`: started, finished, skipped, missed with
  blocked_by, waited, slept, woke, settings changed with who. Persisted in `~/.vitulus/autonomy_journal.jsonl`,
  bounded.

`timeline` and `journal` are published apart, latched: `/scheduler/timeline`, `/scheduler/journal`.

## 3. Behaviour changes

- Hold semantics stated in UI: stops new starts only; a running mission finishes unless stopped.
- `Sleep now` goes through master (`/scheduler/sleep_now` `{"mode":"timed|charged","ms":n}`), respecting
  off_dock, mission and manual-busy gates; `sleep_ignore_ssh` and `sleep_ignore_ui` apply. Before a sleep with
  ignored logins: 60 s countdown on `/scheduler/sleep_countdown` that a person can cancel. The raw PM topics
  remain for service use, moved under Advanced in Settings > Power.
- `/scheduler/run_now` not needed: Run stays in Programs. Add "Skip next" and "Run next planned now".
- Events worth a person go to `/scheduler/notify` JSON `{"level","code","vars","program"}`: missed, stranded,
  run failed, sleep failed. Agent or Discord picks it up.

## 4. Safety

Settings never relax firmware or safety limits. Battery start can't be set under the return level or under
20 %. `rain_mode` never affects anything but rain. Validation is in master, UI validates too. A running
mission keeps values that would alter it mid-run where that is unsafe; say which in `schema`.
Calendar behaviour with default settings must equal today's, with one intended change: rain now applies to
manual runs. Prove with existing calendar sims.

## 5. Worker rules

No commit, push, restarts, publishing to live ROS. Worktrees `<repo>/.claude/worktrees/autonomy`, branch
`autonomy-2026-11`, baseline staged in the index, so `git diff` shows only your work: never `git add`.
Tests without a ROS master, once at the end, per file. UI text through i18n en + cs in the same step, help en +
cs. Notebook first, phone must work. Panel anatomy: one layout, no left stripe, nothing jumps, state
remembered. `tools/build_index.py` is never run. Questions via SendMessage to "main".
