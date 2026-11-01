"""A program starts on ITS map (vitulus-field#46).

Every start of a program - Run and Resume in the UI, the calendar, the agent -
is one message on /web_plan/program_select[_resume], so the planner is the one
place that can make sure the program's map is loaded, verified and ready BEFORE
the program reaches the mission, i.e. before the robot undocks.

No ROS in here: `plan_start` and `switch_blocker` are pure decisions, and
`MapGate` runs the wait against an `env` object (the planner node, a fake in
test/test_program_map.py).

Program reference on the select topics:
    'Name'          as always: the program of the active map, else the one map
                    that has a program of this name
    'SITE|||Name'   exactly that program (same name on two maps)

/web_plan/program_start_state (latched JSON): what became of the last start
that needed the map, for the UI, master_controller and the agent:
    {seq, t, verb, ref, program, site, state, code, params, text_en, text_cs}
state: 'switching' (map being loaded) | 'ready' (program handed to the mission)
       | 'failed' (nothing started, the robot stays where it is).
"""
import json
import threading

SEP = '|||'
LEGACY_MARK = '***env*'
# map_name of data older than vitulus-field#46: names no map.
NO_OWNER = ('', 'SITE', 'New')

SWITCHING = 'switching'
READY = 'ready'
FAILED = 'failed'

# The mission stands, but only Reset gets it out of there: the robot may be
# anywhere and a program sent now would be dropped.
MISSION_WAITS = ('stopped', 'terminal error')

# (en, cs). Every refusal says what is wrong and what to do about it. The UI
# translates by code (mapview.prog.mapgate.<code>), these go to the log, the
# agent and the calendar issue.
REASONS = {
    'switching': (
        "the map '{site}' is being loaded for program '{busy}' - try again in a moment",
        "právě se načítá mapa {site} pro program {busy}, zkus to za chvíli"),
    'not_found': (
        "program '{program}' is on no map",
        "program {program} není na žádné mapě"),
    'not_on_map': (
        "map '{site}' has no program '{program}'",
        "mapa {site} nemá program {program}"),
    'ambiguous': (
        "program '{program}' is on several maps ({sites}) - say which one",
        "program {program} je na více mapách ({sites}), řekni na které"),
    'legacy_program': (
        "program '{program}' belongs to the old map '{site}', which this robot no "
        "longer uses - create it again on a current map",
        "program {program} patří staré mapě {site}, kterou robot už nepoužívá; "
        "založ ho znovu na aktuální mapě"),
    'not_docked': (
        "map '{site}' can only be loaded in the dock, the robot is out - bring it "
        "to the dock and press Run again",
        "mapu {site} jde načíst jen v doku a robot je venku; dojeď do doku a "
        "spusť program znovu"),
    'dock_busy': (
        "map '{site}' cannot be loaded while the robot is docking or undocking",
        "mapu {site} nejde načíst, když robot zrovna zajíždí do doku nebo vyjíždí"),
    'dock_unknown': (
        "map '{site}' not loaded: the dock state is not known",
        "mapa {site} se nenačetla: není znám stav doku"),
    'mission_running': (
        "map '{site}' cannot be loaded while a program is running - stop it first",
        "mapu {site} nejde načíst, když běží program; nejdřív ho zastav"),
    'mission_unknown': (
        "map '{site}' not loaded: the mission does not answer",
        "mapa {site} se nenačetla: mise neodpovídá"),
    'mission_waits_reset': (
        "map '{site}' cannot be loaded: the mission waits for Reset",
        "mapu {site} nejde načíst: mise čeká na Reset"),
    'mapping_running': (
        "map '{site}' cannot be loaded while a map is being recorded",
        "mapu {site} nejde načíst, když se nahrává mapa"),
    'no_status': (
        "map '{site}' not loaded: the map manager does not answer",
        "mapa {site} se nenačetla: správce map neodpovídá"),
    'serve_failed': (
        "map '{site}' could not be loaded: {detail}",
        "mapu {site} se nepodařilo načíst: {detail}"),
    'timeout_serve': (
        "map '{site}' did not become active within {timeout} s",
        "mapa {site} se do {timeout} s nezapnula"),
    'timeout_planner': (
        "the zones and programs of map '{site}' were not ready within {timeout} s",
        "zóny a programy mapy {site} nebyly do {timeout} s připravené"),
    'timeout_navi': (
        "navigation did not take over map '{site}' within {timeout} s",
        "navigace nepřevzala mapu {site} do {timeout} s"),
    'pose_mismatch': (
        "after loading map '{site}' the robot is {error} m away from the dock of "
        "that map - check its DOCKED point",
        "po načtení mapy {site} je robot {error} m od doku té mapy; zkontroluj "
        "její bod DOCKED"),
    'no_dock_point': (
        "map '{site}' has no DOCK point, the robot could not get home - add it first",
        "mapa {site} nemá bod DOCK, robot by se nevrátil domů; nejdřív ho přidej"),
    'no_docked_point': (
        "map '{site}' has no DOCKED point, the robot cannot place itself on it in "
        "the dock - dock it on that map and save the point first",
        "mapa {site} nemá bod DOCKED, robot se na ní v doku neumístí; zadokuj ho "
        "na té mapě a bod ulož"),
    'cancelled': (
        "start of program '{program}' cancelled",
        "spuštění programu {program} zrušeno"),
    'program_unusable': (
        "program '{program}' on map '{site}' cannot run: {detail}",
        "program {program} na mapě {site} nejde spustit: {detail}"),
}


def reason_text(code, lang='en', **params):
    """The sentence for a refusal code. Never raises: a missing parameter must
    not turn a refusal into a dead worker."""
    pair = REASONS.get(code)
    if pair is None:
        return code
    template = pair[1] if lang == 'cs' else pair[0]

    class _Keep(dict):
        def __missing__(self, key):
            return '?'
    return template.format_map(_Keep(params))


def parse_ref(ref):
    """(site or None, name) of a program reference."""
    ref = (ref or '').strip()
    if SEP in ref:
        site, name = ref.split(SEP, 1)
        return (site.strip() or None), name.strip()
    return None, ref


def make_ref(site, name):
    return '{}{}{}'.format(site, SEP, name) if site else name


def is_legacy_name(map_name):
    return LEGACY_MARK in (map_name or '')


def legacy_is_indoor(map_name):
    return (map_name or '').split(LEGACY_MARK)[-1] == 'INDOOR'


def catalog_from_status(status):
    """{site: [program names]} from a /mapping_manager/status dict, or None when
    the status says nothing about sites."""
    sites = (status or {}).get('sites')
    if not isinstance(sites, list):
        return None
    out = {}
    for info in sites:
        if isinstance(info, dict) and info.get('name'):
            out[info['name']] = list(info.get('programs') or [])
    return out


class Plan(object):
    """direct: run it now, its map is on (or there is nothing to decide).
    switch: load `site` first.  refuse: `code`, nothing happens."""
    DIRECT, SWITCH, REFUSE = 'direct', 'switch', 'refuse'

    def __init__(self, kind, name, site=None, code=None, **params):
        self.kind, self.name, self.site = kind, name, site
        self.code, self.params = code, params
        self.verify_only = False     # SWITCH of a map that is on: no load, wait

    def __repr__(self):
        return 'Plan(%s, %s, site=%s, code=%s)' % (self.kind, self.name, self.site, self.code)


def plan_start(ref, legacy_mode, served, active_map, list_programs, catalog):
    """Which map a start needs.

    legacy_mode:   navi_man legacy_maps=true (map_name '<name>***env*<ENV>')
    served:        the site being served, or None
    active_map:    navi_man active_map
    list_programs: {name: map_name} of the planner's list, as far as it may be
                   used (empty while it belongs to another site)
    catalog:       {site: [names]} of all maps, None = not known yet
    """
    site, name = parse_ref(ref)
    owner = list_programs.get(name)

    if legacy_mode:
        if owner is None:
            return Plan(Plan.DIRECT, name)     # planner says "not found"
        if is_legacy_name(owner) and owner != active_map:
            return Plan(Plan.SWITCH, name, site=owner)
        return Plan(Plan.DIRECT, name)

    if owner is not None and is_legacy_name(owner):
        # Site-native navi rejects a legacy map after the undock: say it here.
        return Plan(Plan.REFUSE, name, code='legacy_program',
                    site=owner.split(LEGACY_MARK)[0])

    if site is None:
        if owner is not None and served:
            return Plan(Plan.DIRECT, name, site=served)
        if catalog is None:
            return Plan(Plan.DIRECT, name)     # as before: planner decides
        if served and name in catalog.get(served, ()):
            return Plan(Plan.DIRECT, name, site=served)
        owners = sorted(s for s, names in catalog.items() if name in names)
        if not owners:
            return Plan(Plan.DIRECT, name)     # planner says "not found"
        if len(owners) > 1:
            return Plan(Plan.REFUSE, name, code='ambiguous',
                        sites=', '.join(owners))
        site = owners[0]
    elif catalog is not None and name not in catalog.get(site, ()):
        if site != served or owner is None:
            return Plan(Plan.REFUSE, name, site=site, code='not_on_map')

    if site == served:
        return Plan(Plan.DIRECT, name, site=site)
    return Plan(Plan.SWITCH, name, site=site)


def switch_blocker(dock_status, in_dock, mission_status, mission_busy, mapping_running):
    """Why the map must not change now: a code, or None. Unknown = no: the map
    changes only on a robot known to sit in the dock with nothing running."""
    status = (mission_status or '').strip().lower()
    if status in MISSION_WAITS:
        return 'mission_waits_reset'
    if mission_busy is None:
        return 'mission_unknown'
    if mission_busy:
        return 'mission_running'
    if mapping_running:
        return 'mapping_running'
    if dock_status in (1, 2):
        return 'dock_busy'
    if dock_status is None or in_dock is None:
        return 'dock_unknown'
    if dock_status != 0 or not in_dock:
        return 'not_docked'
    return None


def needs_verify(ready, site, dock_status, tol_m):
    """The map is on, but was it ever taken over? A switch that failed leaves it
    served, so the next Run must not go straight to the undock. Only in the dock:
    a robot that is out starts as it always did, the map does not change there.
    None = navi has not said anything yet: nothing to go by."""
    if not isinstance(ready, dict) or dock_status != 0:
        return False
    if not ready.get('ready') or ready.get('site') != site:
        return True
    err = ready.get('dock_err_m')
    return err is not None and err > tol_m


def navi_ready(ready, target, legacy_mode):
    """(ok, code, params) of a /navi_manager/map_ready dict for `target`.
    Still waiting = (False, None, {}); ok=False with a code = will not get better
    by waiting (the worker only uses it at the timeout)."""
    if not isinstance(ready, dict) or not ready.get('ready'):
        return False, None, {}
    if legacy_mode:
        if ready.get('active_map') != target:
            return False, None, {}
    elif ready.get('site') != target:
        return False, None, {}
    if not legacy_mode and ready.get('has_dock') is False:
        return False, 'no_dock_point', {}
    if not legacy_mode and ready.get('has_docked') is False:
        # No dock hold without it: the robot would leave with the old map's pose.
        return False, 'no_docked_point', {}
    return True, None, {}


class MapGate(object):
    """One start at a time: load the map, verify it, then hand the program over.
    Runs in its own thread (rospy delivers a callback per topic; a load takes
    tens of seconds and Stop must still get through)."""

    POLL_S = 0.2
    SETTLE_S = 2.0    # readiness has to hold this long (dock hold takes ~1 s)

    def __init__(self, env, timeout_s=120.0, pose_tol_m=0.5):
        self.env = env
        self.timeout_s = float(timeout_s)
        self.pose_tol_m = float(pose_tol_m)
        self._lock = threading.Lock()
        self._busy = None            # dict of the running start
        self._cancel = threading.Event()
        self._seq = 0

    def busy(self):
        with self._lock:
            return dict(self._busy) if self._busy else None

    def cancel(self):
        """Stop pressed: nothing starts. The map stays as far as it got."""
        with self._lock:
            if self._busy:
                self._cancel.set()

    def _report(self, state, verb, ref, program, site, code=None, notes=None, **params):
        self._seq += 1
        params = dict(params, program=program, site=site)
        report = {
            'seq': self._seq, 't': self.env.now(), 'verb': verb, 'ref': ref,
            'program': program, 'site': site, 'state': state, 'code': code,
            'params': {k: v for k, v in params.items() if k not in ('program', 'site')},
            'text_en': reason_text(code, 'en', **params) if code else '',
            'text_cs': reason_text(code, 'cs', **params) if code else '',
        }
        if notes:
            report['notes'] = notes     # what this start does differently, by zone
        self.env.report(report)

    def refuse(self, verb, ref, program, site, code, **params):
        self._report(FAILED, verb, ref, program, site, code, **params)
        self.env.log("Program '{}' not started: {}.".format(
            program, reason_text(code, 'en', program=program, site=site, **params)))

    def refuse_busy(self, verb, ref, program, site, running):
        """A second start while a map loads. The state topic ends on the running
        start again: master_controller follows that one, not the refused one."""
        self.refuse(verb, ref, program, site, 'switching', busy=running['program'])
        self._report(SWITCHING, running['verb'], running['ref'], running['program'],
                     running['site'])

    def ready(self, verb, ref, program, site):
        notes = self.env.start_notes() if hasattr(self.env, 'start_notes') else None
        self._report(READY, verb, ref, program, site, notes=notes)

    def submit(self, plan, ref, verb, legacy_mode, start):
        """Start the worker for a Plan.SWITCH. `start` = the caller's hand-over,
        (ok, detail). False when a start is already running."""
        running = None
        with self._lock:
            if self._busy:
                running = dict(self._busy)
            else:
                self._busy = {'program': plan.name, 'site': plan.site, 'ref': ref,
                              'verb': verb}
                self._cancel.clear()
        if running:
            # The calendar and the UI may repeat themselves: same start = no news.
            if running['ref'] != ref:
                self.refuse_busy(verb, ref, plan.name, plan.site, running)
            return False
        worker = threading.Thread(target=self._run, name='program_map_gate',
                                  args=(plan, ref, verb, legacy_mode, start))
        worker.daemon = True
        worker.start()
        return True

    def _run(self, plan, ref, verb, legacy_mode, start):
        name, site = plan.name, plan.site
        try:
            code, params = self._switch(plan, ref, verb, legacy_mode)
            if code is None:
                ok, detail = start(name, verb)
                if ok:
                    self.ready(verb, ref, name, site)
                else:
                    self.refuse(verb, ref, name, site, 'program_unusable', detail=detail)
            elif code == 'cancelled':
                self.refuse(verb, ref, name, site, code)
            else:
                self.refuse(verb, ref, name, site, code, **params)
        except Exception as exc:  # noqa: BLE001 - the worker must always report
            self.env.log("Map gate failed: {}".format(exc))
            self.refuse(verb, ref, name, site, 'serve_failed', detail=str(exc)[:160])
        finally:
            with self._lock:
                self._busy = None

    def _switch(self, plan, ref, verb, legacy_mode):
        """Load + verify. (None, {}) = ready, else (code, params)."""
        env, site = self.env, plan.site
        blocker = switch_blocker(*env.robot_state())
        if blocker:
            return blocker, {}
        if not legacy_mode and not env.status_seen():
            return 'no_status', {}
        self._report(SWITCHING, verb, ref, plan.name, site)
        if plan.verify_only:
            stamp = env.now()
        else:
            env.log("Program '{}': loading map '{}' before the undock.".format(plan.name, site))
            stamp = env.request_map(site, legacy_mode)
        deadline = env.now() + self.timeout_s
        phase = 'timeout_serve'
        good_since = None
        while True:
            if self._cancel.is_set():
                return 'cancelled', {}
            # Conditions are re-checked all the way: the robot being pushed out
            # or a mission started meanwhile ends the switch.
            blocker = switch_blocker(*env.robot_state())
            if blocker:
                return blocker, {}
            error = env.serve_error(stamp)
            if error and not legacy_mode:
                return 'serve_failed', {'detail': error}
            ok = False
            if env.map_on(site, legacy_mode):
                phase = 'timeout_planner'
                if env.planner_ready(site, legacy_mode):
                    phase = 'timeout_navi'
                    ok, code, params = navi_ready(env.navi_state(), site, legacy_mode)
                    if code:
                        return code, params
                    pose_err = (env.navi_state() or {}).get('dock_err_m')
                    if ok and pose_err is None and not legacy_mode:
                        ok = False      # no pose on the new map yet: not verified
                    if ok and pose_err is not None and pose_err > self.pose_tol_m:
                        ok = False
                        phase = 'pose_mismatch'
            if ok:
                now = env.now()
                good_since = now if good_since is None else good_since
                if now - good_since >= self.SETTLE_S:
                    return None, {}
            else:
                good_since = None
            if env.now() >= deadline:
                if phase == 'pose_mismatch':
                    err = (env.navi_state() or {}).get('dock_err_m')
                    return phase, {'error': '%.1f' % float(err or 0.0)}
                return phase, {'timeout': int(self.timeout_s)}
            env.sleep(self.POLL_S)


def dumps(state):
    return json.dumps(state, ensure_ascii=False)
