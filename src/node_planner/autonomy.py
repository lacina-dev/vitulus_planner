"""Settings of the Autonomy page, without ROS (docs/AUTONOMY.md).

master_controller owns ~/.vitulus/autonomy.json. A missing file or key means the
yaml default, a bad file is kept aside. FIELDS is the one list of the keys: what
the UI is told (schema), what is accepted (clean) and where each default comes
from. Applying a value to the running logic is the master's _apply_settings.
"""
import datetime
import json
import os
import shutil
import threading
import time

VERSION = 1

RAIN_MODES = ('always', 'calendar', 'ignore')
STAY_AWAKE_MAX_S = 7 * 24 * 3600.0
NO_MOW_MAX_DAYS = 366
BATTERY_START_FLOOR = 20
BATTERY_START_MARGIN = 5


def _f(key, kind, group, param=None, fallback=None, lo=None, hi=None, unit='',
       advanced=False, **more):
    d = {'key': key, 'type': kind, 'group': group, 'param': param,
         'fallback': fallback, 'min': lo, 'max': hi, 'unit': unit,
         'advanced': advanced}
    d.update(more)
    return d


# param: the ~param of master_controller.yaml that gives the default.
# div: param value / div = the setting. invert: the param says the opposite.
FIELDS = [
    # --- plan ---
    _f('window_enabled', 'bool', 'plan', 'allowed_window_enabled', True),
    _f('window_start', 'time', 'plan', 'allowed_window_start', '07:00'),
    _f('window_end', 'time', 'plan', 'allowed_window_end', '21:00'),
    _f('window_min_left_min', 'float', 'plan', 'start_min_window_left_min', 30.0,
       0, 240, 'min', True),
    _f('catchup_max_h', 'float', 'plan', 'catchup_max_h', 4.0, 0.5, 24, 'h', True),
    _f('skip_next', 'slot', 'plan'),
    _f('no_mow_until', 'date', 'plan'),
    # --- rain ---
    _f('rain_mode', 'enum', 'rain', 'rain_mode', 'always', options=list(RAIN_MODES)),
    _f('rain_after_min', 'int', 'rain', 'rain_after_min', 60, 0, 60, 'min', True,
       step=10),
    _f('rain_forecast_min', 'int', 'rain', 'rain_forecast_min', 30, 10, 30, 'min',
       True, step=10),
    _f('rain_retry_h', 'float', 'rain', 'weather_sleep_duration_h', 4.0, 0.25, 24,
       'h', True),
    _f('rain_return_home', 'bool', 'rain', 'rain_return_home', True),
    _f('rain_stale_ok', 'bool', 'rain', 'rain_stale_ok', True, advanced=True),
    # --- battery ---
    _f('battery_start_pct', 'int', 'battery', 'battery_recovery_min_capacity', 40,
       BATTERY_START_FLOOR, 80, '%'),
    # A run out now keeps the level it left with: it decides how far from home
    # the robot may get, and the way back was planned on it.
    _f('battery_return_pct', 'int', 'battery', 'battery_return_pct', 25, 20, 75, '%',
       mid_run='next_run'),
    _f('battery_resume_pct', 'int', 'battery', 'battery_resume_after_low_pct', 80,
       20, 100, '%'),
    _f('charge_sleep_max_h', 'float', 'battery', 'charge_sleep_max_h', 6.0, 0.5, 12,
       'h', True),
    # --- sleep ---
    _f('sleep_enabled', 'bool', 'sleep', 'sleep_enabled', True),
    _f('sleep_ignore_ssh', 'bool', 'sleep', 'sleep_check_logins', False, invert=True),
    _f('sleep_ignore_ui', 'bool', 'sleep', 'sleep_check_ui', False, invert=True),
    _f('idle_sleep_threshold_min', 'float', 'sleep', 'idle_sleep_threshold_s', 20.0,
       5, 240, 'min', True, div=60.0),
    _f('wake_before_min', 'float', 'sleep', None, 12.0, 2, 60, 'min', True),
    _f('sleep_after_run', 'bool', 'sleep', 'sleep_after_run', True),
    _f('stay_awake_until', 'datetime', 'sleep'),
    # --- recovery ---
    _f('temp_wait_h', 'float', 'recovery', 'temp_sleep_duration_h', 1.0, 0.25, 12,
       'h', True),
    _f('temp_spike_wait_min', 'float', 'recovery', 'temp_spike_wait_min', 5.0, 1, 120,
       'min', True),
    _f('error_retry_max', 'int', 'recovery', 'critical_retry_max', 3, 1, 10, '', True),
    _f('resume_max', 'int', 'recovery', 'resume_max_attempts', 12, 1, 50, '', True),
    _f('run_max_age_h', 'float', 'recovery', 'run_max_age_h', 48.0, 1, 168, 'h', True),
    _f('off_dock_wait_min', 'float', 'recovery', 'off_dock_wait_s', 20.0, 1, 240,
       'min', True, div=60.0),
]
BY_KEY = {f['key']: f for f in FIELDS}

# Shown, never settable. Only what the robot really enforces, with where it does;
# values are read from nowhere here and feed no decision. The agent's battery
# warning levels (senses.yaml) are messages, not limits, and are not here.
LOCKED = [
    {'key': 'mower_temp_limit_c', 'value': 70, 'unit': 'C', 'locked_why': 'firmware',
     'source': 'mower firmware TEMP_MAX; safety.yaml mower_max_temp'},
    {'key': 'motor_temp_limit_c', 'value': 70, 'unit': 'C', 'locked_why': 'safety_temp',
     'source': 'safety.yaml motor_max_temp'},
    {'key': 'mower_temp_hard_c', 'value': 100, 'unit': 'C', 'locked_why': 'safety_temp',
     'source': 'safety.yaml mower_hard_temp'},
    {'key': 'tilt_limit_deg', 'value': 30, 'unit': 'deg', 'locked_why': 'tilt',
     'source': 'safety.yaml max_tilt_x, max_tilt_y'},
    {'key': 'stop_button', 'value': True, 'unit': '', 'locked_why': 'stop',
     'source': 'STOP button, mission STOP_MONITOR'},
    {'key': 'blade_rules', 'value': True, 'unit': '', 'locked_why': 'blade',
     'source': 'map rules, map_rules_guard and rule_gate'},
]
LOCKED_KEYS = {x['key']: x for x in LOCKED}


def defaults(get_param):
    """The yaml defaults; get_param(name, fallback) as rospy reads ~name."""
    out = {}
    for f in FIELDS:
        if f['param'] is None:
            out[f['key']] = f['fallback']
            continue
        fallback = f['fallback']
        if 'div' in f:
            fallback = fallback * f['div']
        if f.get('invert'):
            fallback = not fallback
        value = get_param('~' + f['param'], fallback)
        if 'div' in f:
            value = value / f['div']
        if f.get('invert'):
            value = not value
        out[f['key']] = value
    out['window_start'] = str(out['window_start'])
    out['window_end'] = str(out['window_end'])
    out['wake_before_min'] = (get_param('~wake_before_program_s', 600.0)
                              + get_param('~boot_time_buffer_s', 120.0)) / 60.0
    return out


def schema():
    out = {}
    for f in FIELDS:
        entry = {'type': f['type'], 'min': f['min'], 'max': f['max'], 'unit': f['unit'],
                 'group': f['group'], 'advanced': f['advanced'], 'locked': False,
                 'needs_restart': False}
        for k in ('step', 'options', 'mid_run'):
            if k in f:
                entry[k] = f[k]
        out[f['key']] = entry
    for x in LOCKED:
        out[x['key']] = {'type': 'bool' if isinstance(x['value'], bool) else 'int',
                         'min': None, 'max': None, 'unit': x['unit'],
                         'group': 'locked', 'advanced': False, 'locked': True,
                         'needs_restart': False, 'locked_why': x['locked_why'],
                         'source': x['source']}
    return out


def locked_values():
    return {x['key']: x['value'] for x in LOCKED}


def parse_hhmm(s):
    try:
        h, m = str(s).split(':')[:2]
        h, m = int(h), int(m)
    except ValueError:
        return None
    return (h, m) if 0 <= h <= 23 and 0 <= m <= 59 else None


def parse_date(s):
    try:
        return datetime.datetime.strptime(str(s), "%Y-%m-%d").date()
    except ValueError:
        return None


def parse_datetime(s):
    """Local 'YYYY-MM-DDTHH:MM' (seconds are dropped)."""
    try:
        return datetime.datetime.strptime(str(s)[:16], "%Y-%m-%dT%H:%M")
    except ValueError:
        return None


def _clean_value(f, value, now):
    """(value, None) or (None, reason) for one key alone."""
    kind = f['type']
    if kind == 'bool':
        return (value, None) if isinstance(value, bool) else (None, 'type')
    if kind == 'enum':
        return (value, None) if value in f['options'] else (None, 'option')
    if kind == 'time':
        hm = parse_hhmm(value) if isinstance(value, str) else None
        return ("%02d:%02d" % hm, None) if hm else (None, 'type')
    if kind == 'date':
        if value is None:
            return None, None
        day = parse_date(value)
        if day is None:
            return None, 'type'
        if day < now.date():
            return None, 'past'
        if (day - now.date()).days > NO_MOW_MAX_DAYS:
            return None, 'too_far'
        return day.strftime("%Y-%m-%d"), None
    if kind == 'datetime':
        if value is None:
            return None, None
        at = parse_datetime(value)
        if at is None:
            return None, 'type'
        if at <= now:
            return None, 'past'
        if (at - now).total_seconds() > STAY_AWAKE_MAX_S:
            return None, 'too_far'
        return at.strftime("%Y-%m-%dT%H:%M"), None
    if kind == 'slot':
        # true: the master pins the next slot. false / null: clear. A pinned
        # value coming back from the file is the slot itself.
        if value is True or value is None or value is False:
            return (True if value else None), None
        return (value, None) if parse_datetime(value) else (None, 'type')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, 'type'
    if value != value or value in (float('inf'), float('-inf')):
        return None, 'type'
    if kind == 'int':
        if int(value) != value:
            return None, 'type'
        value = int(value)
    else:
        value = float(value)
    if value < f['min'] or value > f['max']:
        return None, 'range'
    step = f.get('step')
    if step and value % step:
        return None, 'range'
    return value, None


# Who is judged together: a rule is only asked of a change that touches its group,
# so an old value (a short window from /scheduler/set_window) cannot block the rest.
GROUPS = (('battery_start_pct', 'battery_return_pct', 'battery_resume_pct'),
          ('window_enabled', 'window_start', 'window_end', 'window_min_left_min'),
          ('catchup_max_h', 'wake_before_min'))


def _group(key):
    return next((g for g in GROUPS if key in g), (key,))


def cross_check(values, boot_grace_s=300.0):
    """Rules between keys over the merged values: {key: reason}. Settings can
    never go under what keeps a robot able to come home, and a start level over
    80 % could wait for ever on a full pack that reads less."""
    bad = {}
    start = values['battery_start_pct']
    if start < BATTERY_START_FLOOR:
        bad['battery_start_pct'] = 'battery_start_under_20'
    elif start < values['battery_return_pct'] + BATTERY_START_MARGIN:
        bad['battery_start_pct'] = 'battery_start_under_return'
    if values['battery_resume_pct'] < start:
        bad['battery_resume_pct'] = 'battery_resume_under_start'
    begin = parse_hhmm(values['window_start'])
    end = parse_hhmm(values['window_end'])
    if values['window_start'] == values['window_end']:
        bad['window_end'] = 'window_equal'
    elif begin and end and values['window_enabled']:
        length = ((end[0] - begin[0]) * 60 + end[1] - begin[1]) % 1440
        if values['window_min_left_min'] >= length:
            bad['window_min_left_min'] = 'window_too_short'
    # A robot that wakes for a slot and is late must still catch it after booting.
    if values['catchup_max_h'] * 3600.0 < values['wake_before_min'] * 60.0 + boot_grace_s:
        bad['catchup_max_h'] = 'catchup_under_wake'
    return bad


def clean(request, current, now, touched=None, boot_grace_s=300.0):
    """Validate a partial change against the current values.

    Returns (cleaned, rejected): cleaned has only the keys asked for; rejected
    is {key: reason}. Anything rejected means nothing is applied: half of a pair
    of limits is worse than none.
    """
    cleaned, rejected = {}, {}
    for key, value in request.items():
        if key in LOCKED_KEYS:
            rejected[key] = 'locked'
            continue
        f = BY_KEY.get(key)
        if f is None:
            rejected[key] = 'unknown'
            continue
        value, why = _clean_value(f, value, now)
        if why:
            rejected[key] = why
        else:
            cleaned[key] = value
    merged = dict(current)
    merged.update(cleaned)
    touched = set(request) if touched is None else set(touched)
    rejected.update({k: v for k, v in cross_check(merged, boot_grace_s).items()
                     if k not in rejected and touched & set(_group(k))})
    return cleaned, rejected


def clean_stored(stored, base, now):
    """Keys of a file that are usable. A one-shot in the past is simply over; a
    key that does not pass is dropped. Returns (values, bad keys)."""
    out, bad = {}, []
    for key, value in stored.items():
        f = BY_KEY.get(key)
        if f is None:
            bad.append(key)
            continue
        if f['type'] in ('date', 'datetime', 'slot'):
            ok = value is None or (
                parse_date(value) if f['type'] == 'date' else parse_datetime(value))
            if ok:
                out[key] = value
            else:
                bad.append(key)
            continue
        value, why = _clean_value(f, value, now)
        if why:
            bad.append(key)
        else:
            out[key] = value
    merged = dict(base)
    merged.update(out)
    for key in cross_check(merged):
        # The group is what is wrong: its keys go back to their defaults.
        for k in _group(key):
            if k in out:
                del out[k]
                bad.append(k)
    return out, bad


class Journal(object):
    """Last events, ~/.vitulus/autonomy_journal.jsonl. Appended, cut back when it
    has grown; a line a power cut tore is skipped."""
    KEEP = 100
    MAX_LINES = 400

    def __init__(self, path, log=lambda m: None):
        self.path = path
        self.log = log
        self.lock = threading.Lock()
        self.lines = 0
        self.items = []
        try:
            with open(path) as f:
                raw = f.read().splitlines()
        except (OSError, UnicodeDecodeError):
            raw = []
        self.lines = len(raw)
        for line in raw:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if isinstance(item, dict):
                self.items.append(item)
        self.items = self.items[-self.KEEP:]

    def entries(self):
        with self.lock:
            return list(self.items)

    def add(self, item):
        with self.lock:
            self.items = (self.items + [item])[-self.KEEP:]
            try:
                line = json.dumps(item, sort_keys=True, default=str)
                if self.lines >= self.MAX_LINES:
                    self._rewrite()
                else:
                    with open(self.path, 'a') as f:
                        f.write(line + '\n')
                    self.lines += 1
            except (OSError, ValueError, TypeError) as e:
                self.log('journal: %s' % e)

    def _rewrite(self):
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            for item in self.items:
                f.write(json.dumps(item, sort_keys=True, default=str) + '\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        self.lines = len(self.items)


class Store(object):
    """~/.vitulus/autonomy.json: {"version","values","changed_at","by"}."""

    def __init__(self, path, log=lambda m: None):
        self.path = path
        self.log = log

    def load(self):
        """(document, problem): a bad file is moved aside, never lost."""
        empty = {'version': VERSION, 'values': {}, 'changed_at': None, 'by': None}
        if not os.path.exists(self.path):
            return empty, None
        try:
            with open(self.path) as f:
                doc = json.load(f)
            if not isinstance(doc, dict) or not isinstance(doc.get('values'), dict):
                raise ValueError('not a settings file')
            if int(doc.get('version', 0)) > VERSION:
                raise ValueError('newer version')
        except (OSError, ValueError, TypeError) as e:
            aside = self.keep_aside()
            self.log('autonomy.json is bad (%s), kept as %s' % (e, aside))
            return empty, 'unreadable'
        for k, v in empty.items():
            doc.setdefault(k, v)
        return doc, None

    def keep_aside(self):
        """A copy of the file as it is; returns its name."""
        aside = '%s.bad-%d' % (self.path, int(time.time()))
        try:
            os.replace(self.path, aside)
        except OSError:
            return None
        return aside

    def keep_copy(self):
        """As keep_aside, for a file that stays in use."""
        aside = '%s.bad-%d' % (self.path, int(time.time()))
        try:
            shutil.copyfile(self.path, aside)
        except OSError:
            return None
        return aside

    def save(self, doc):
        """Atomic and durable, as the state file: power can go at any moment."""
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(doc, f, indent=2, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        fd = os.open(os.path.dirname(self.path), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
