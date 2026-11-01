#!/usr/bin/env python3
"""A program starts on its own map (vitulus-field#46), off the robot.

REAL: node_planner, mapping_manager, master_controller, mower_unit_smach.
FAKE: navi_man + dock + the rest (fake_world_maps.py). PRIVATE roscore, private
HOME; two maps made from a copy of a real site.

  python3 run_program_map.py [--rounds 3] [scenario ...]

Every scenario proves its order from the event log: the map is loaded and
verified before the first undock command, or no undock command exists at all.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PLANNER = os.path.abspath(os.path.join(HERE, '..', '..'))
SITE_SRC = os.environ.get('SIM_SITE_SRC', os.path.expanduser('~/.vitulus/mapping_v3/10_2026'))
SCENARIOS = ['a_switch', 'a_plain_name', 'a_no_map_on', 'b_same_map', 'c_no_raster', 'c_navi_timeout',
             'c_pose', 'c_no_dock', 'c_planner_timeout', 'd_outside', 'd_mission', 'd_reset_wait',
             'e_calendar', 'e_calendar_fail', 'e_resume', 'e_resume_across', 'f_legacy',
             'g_hold', 'g_hold_calendar', 'h_stop', 'i_ambiguous', 'i_race_second_run',
             'j_edit_guard']


def outer():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenarios', nargs='*')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--port', type=int, default=11830)
    a = ap.parse_args()
    names = a.scenarios or SCENARIOS
    tmp = tempfile.mkdtemp(prefix='mapsim_')
    results = []
    port = a.port
    for r in range(a.rounds):
        for name in names:
            d = os.path.join(tmp, '%d_%s' % (r, name))
            os.makedirs(d)
            env = dict(os.environ, SIM_INNER='1', SIM_PORT=str(port), SIM_TMP=d,
                       SIM_PLANNER=os.environ.get('SIM_PLANNER_UNDER_TEST', PLANNER),
                       SIM_NAVI_TREE=_tree('vitulus_navi'),
                       SIM_MAPPING_TREE=_tree('vitulus_mapping'),
                       SIM_OLD='', TZ='Europe/Prague')
            t0 = time.time()
            rc = subprocess.call([sys.executable, os.path.abspath(__file__), name], env=env)
            results.append((r, name, rc))
            print('=== round %d %-20s %s (%.0f s)' % (
                r, name, 'PASS' if rc == 0 else 'FAIL', time.time() - t0), flush=True)
    bad = [x for x in results if x[2]]
    print('\nSUMMARY: %d scenario runs, %d failed' % (len(results), len(bad)))
    for x in bad:
        print('  FAIL round %d %s' % x[:2])
    if bad:
        print('logs:', tmp)
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return 1 if bad else 0


def _tree(repo):
    parts = PLANNER.split(os.sep)
    i = parts.index('vitulus_planner')
    return os.sep.join(parts[:i] + [repo] + parts[i + 1:])


if not os.environ.get('SIM_INNER'):
    sys.exit(outer())

sys.path.insert(0, HERE)
sys.argv = [sys.argv[0]] + sys.argv[1:]
import run_sim as R              # noqa: E402
import rospy                     # noqa: E402
import yaml                      # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

check, D, daily, start = R.check, R.D, R.daily, R.start
HOME = R.HOME
SITES = os.path.join(HOME, '.vitulus', 'mapping_v3')
MAPPING_T = os.environ['SIM_MAPPING_TREE']


def make_site(name, shift, programs):
    d = os.path.join(SITES, name)
    shutil.copytree(SITE_SRC, d, ignore=shutil.ignore_patterns('logs', '*.bak*'))
    m = yaml.safe_load(open(os.path.join(d, 'manifest.yaml')))
    m['site'] = name
    yaml.safe_dump(m, open(os.path.join(d, 'manifest.yaml'), 'w'))
    y = yaml.safe_load(open(os.path.join(d, 'programs.yaml')))
    base = y['programs'][0]
    y['programs'] = [dict(base, name=n, last_result=lr, zone_names=z)
                     for n, z, lr in programs]
    yaml.safe_dump(y, open(os.path.join(d, 'programs.yaml'), 'w'))
    dp = os.path.join(d, 'datum.yaml')
    dt = yaml.safe_load(open(dp))
    if 'utm_e' in dt:
        dt['utm_e'] = float(dt['utm_e']) + shift
    yaml.safe_dump(dt, open(dp, 'w'))
    return d


class Sim(R.Sim):
    def __init__(self, timeout=30.0, serve='BBB', legacy=False, **kw):
        self.map_timeout = timeout
        self.first_serve = serve
        self.legacy = legacy
        self.ss = {}
        self.ss_log = []
        self.loaded = ''
        self.mapping = {}
        os.makedirs(SITES, exist_ok=True)
        if not os.path.isdir(os.path.join(SITES, 'AAA')):
            make_site('AAA', 0.0, [('Test', ['Zone1'], ''), ('OnlyA', ['ZoneA'], '')])
            make_site('BBB', 5.0, [('B1Test', ['Zone1'], ''), ('Test', ['ZoneB'], '')])
            make_site('DDD', 7.0, [('Solo', ['Zone1'], '')])
            noraster = make_site('CCC', 9.0, [('NoRaster', ['Zone1'], '')])
            shutil.rmtree(os.path.join(noraster, 'rasters'))
            os.remove(os.path.join(noraster, 'combined.json'))
        R.Sim.__init__(self, **kw)

    def load_params(self):
        R.Sim.load_params(self)
        rospy.set_param('/node_planner/program_map_timeout_s', self.map_timeout)
        rospy.set_param('/navi_manager/legacy_maps', self.legacy)
        rospy.set_param('/navi_manager/site_native', not self.legacy)

    def spawn_world(self):
        if not hasattr(self, 'p_serve'):
            self.p_serve = rospy.Publisher('/mapping_manager/serve_site', String, queue_size=1, latch=True)
            self.p_sel = rospy.Publisher('/web_plan/program_select', String, queue_size=1)
            self.p_res = rospy.Publisher('/web_plan/program_select_resume', String, queue_size=1)
            rospy.Subscriber('/web_plan/program_start_state', String, self._ss)
            rospy.Subscriber('/web_plan/planner_loaded', String, self._s('loaded'))
            rospy.Subscriber('/mapping_manager/status', String, self._map)
        self.spawn('world', [sys.executable, os.path.join(HERE, 'fake_world_maps.py')])
        env = {'PYTHONPATH': os.path.join(MAPPING_T, 'src') + ':' + os.environ.get('PYTHONPATH', '')}
        self.spawn('mapping', [sys.executable, os.path.join(MAPPING_T, 'nodes', 'mapping_manager'),
                               '__name:=mapping_manager'], env)
        env = {'PYTHONPATH': os.path.join(R.PLANNER_T, 'src') + ':' + os.path.join(MAPPING_T, 'src')
                             + ':' + os.environ.get('PYTHONPATH', '')}
        self.spawn('planner', [sys.executable, os.path.join(R.PLANNER_T, 'nodes', 'node_planner'),
                               '__name:=node_planner'], env)

    def _ss(self, msg):
        self.ss = json.loads(msg.data)
        self.ss_log.append(self.ss)

    def _map(self, msg):
        self.mapping = json.loads(msg.data)

    def served(self):
        return (self.mapping.get('serving') or {}).get('site')

    def boot(self):
        """World up, first map on, the planner ready."""
        self.wait(lambda: self.p_serve.get_num_connections() > 0, 30, 'mapping up')
        # mapping_manager serves the newest site itself right after it subscribes:
        # the map of the scenario has to come after that, not race with it.
        self.wait(lambda: self.mapping.get('sites'), 30, 'status')
        if self.first_serve:
            self.p_serve.publish(String(self.first_serve))
            ok = self.wait(lambda: self.loaded == self.first_serve, 90, 'first map')
            check('first map on', ok, self.loaded)
        else:
            self.wait(lambda: self.mapping.get('sites'), 30, 'status')
        self.wait(lambda: self.p_sel.get_num_connections() > 0, 20, 'planner listens')
        time.sleep(1.0)      # status + map_ready reached the planner
        self.mark = time.time()

    def run_prog(self, ref, resume=False):
        (self.p_res if resume else self.p_sel).publish(String(ref))

    def ev(self, kind=None):
        return [e for e in self.events(kind) if e['t'] >= self.mark]

    def undocks(self):
        return [e for e in self.ev('dock_program') if e['action'] == 'UNDOCK']

    def undocked(self, timeout=60):
        return self.wait(lambda: self.undocks(), timeout, 'undock command')

    def failed(self, code, timeout=45):
        ok = self.wait(lambda: self.ss.get('state') == 'failed' and self.ss.get('code') == code,
                       timeout, 'state failed/' + code)
        check('refused ' + code, ok, str(self.ss.get('code')))
        return ok

    def in_order(self, site):
        """serve_site(site) < program_active(map=site) < first UNDOCK; report."""
        e = self.ev()
        keys = [x['kind'] for x in e]
        print('   order:', [(x['kind'], x.get('site') or x.get('map') or x.get('action') or '')
                            for x in e if x['kind'] in ('serve_site', 'serving', 'load_map', 'navi_ready',
                                                        'program_active', 'dock_program')], flush=True)
        for k in ('serve_site', 'serving', 'navi_ready', 'program_active', 'dock_program'):
            if k not in keys:
                check('event ' + k, False)
                return
        t = {k: next(x['t'] for x in e if x['kind'] == k and (k == 'dock_program' or
             site in (x.get('site'), x.get('map')))) for k in
             ('serve_site', 'serving', 'navi_ready', 'program_active')}
        t['dock_program'] = self.undocks()[0]['t']
        check('map first, then the program, then the undock',
              t['serve_site'] < t['serving'] <= t['navi_ready'] < t['program_active'] < t['dock_program'],
              ' '.join('%s+%.1f' % (k, t[k] - t['serve_site']) for k in t))

    def still_docked(self, seconds=3.0):
        time.sleep(seconds)
        check('no undock command', not self.undocks())
        check('robot in dock', json.load(open(os.environ['SIM_STORE']))['s']['docked'])

    def stop(self):
        self.p_stop.publish(Bool(True))


def up(**kw):
    world = kw.pop('world', None)
    s = Sim(**kw)
    if world:
        s.set(**world)
    s.boot()
    return s


def scenario_a_switch():
    """Program of A, B is on, docked: load -> verify -> undock."""
    s = up()
    check('text in the catalog', s.served() == 'BBB')
    s.run_prog('AAA|||OnlyA')
    check('undock', s.undocked())
    s.in_order('AAA')
    check('A is on', s.served() == 'AAA', str(s.served()))
    check('one load', len(s.ev('serve_site')) == 1)
    check('reported ready', s.ss.get('state') == 'ready', str(s.ss))
    check('mission has the program', s.ev('program_active')[-1]['map'] == 'AAA')
    return s


def scenario_a_plain_name():
    """Only its name, the one map that has it is found (agent, old calendar)."""
    s = up()
    s.run_prog('OnlyA')
    check('undock', s.undocked())
    s.in_order('AAA')
    return s


def scenario_a_no_map_on():
    """Mac's case: no map active at all."""
    s = up(serve=None)
    s.p_serve.publish(String('none'))
    s.wait(lambda: not s.served(), 20, 'deactivated')
    time.sleep(1.0)
    s.mark = time.time()
    s.run_prog('B1Test')
    check('undock', s.undocked())
    s.in_order('BBB')
    return s


def scenario_b_same_map():
    """Program of the map that is on: nothing extra."""
    s = up()
    s.run_prog('B1Test')
    check('undock', s.undocked())
    check('no load', not s.ev('serve_site'))
    check('B still on', s.served() == 'BBB')
    s.run_ok = True
    return s


def scenario_c_no_raster():
    """The map cannot be served: no undock, the reason in Czech."""
    s = up()
    s.run_prog('NoRaster')
    s.failed('serve_failed')
    check('names the map in Czech', 'CCC' in s.ss.get('text_cs', '') and 'nepodařilo' in s.ss.get('text_cs', ''),
          s.ss.get('text_cs'))
    s.still_docked()
    return s


def scenario_c_navi_timeout():
    s = up(timeout=8.0, world={'navi_block': True})
    s.run_prog('AAA|||OnlyA')
    s.failed('timeout_navi', 20)
    check('czech', 'navigace' in s.ss.get('text_cs', '').lower(), s.ss.get('text_cs'))
    s.still_docked()
    return s


def scenario_c_pose():
    s = up(timeout=8.0, world={'dock_err': 3.5})
    s.run_prog('AAA|||OnlyA')
    s.failed('pose_mismatch', 20)
    check('czech', '3.5 m' in s.ss.get('text_cs', ''), s.ss.get('text_cs'))
    s.still_docked()
    return s


def scenario_c_no_dock():
    s = up(timeout=8.0)
    s.set(has_dock=False)
    time.sleep(0.5)
    s.mark = time.time()
    s.run_prog('AAA|||OnlyA')
    s.failed('no_dock_point', 20)
    s.still_docked()
    return s


def scenario_c_planner_timeout():
    """The planner cannot finish the map: the site_map never comes."""
    s = up(timeout=8.0)
    # zones of A broken: planner workspace stays incomplete
    with open(os.path.join(SITES, 'AAA', 'zones.geojson'), 'w') as f:
        f.write('{broken')
    s.run_prog('AAA|||OnlyA')
    ok = s.wait(lambda: s.ss.get('state') == 'failed', 30, 'failed')
    check('refused', ok, str(s.ss))
    print('   code:', s.ss.get('code'), s.ss.get('text_cs'), flush=True)
    s.still_docked()
    return s


def scenario_d_outside():
    """Robot out: refused, no load, map untouched."""
    s = up()
    s.set(docked=False)
    time.sleep(0.5)
    s.mark = time.time()
    s.run_prog('AAA|||OnlyA')
    s.failed('not_docked')
    check('in Czech', 'jen v doku' in s.ss.get('text_cs', ''), s.ss.get('text_cs'))
    check('no load', not s.ev('serve_site'))
    check('B still on', s.served() == 'BBB')
    check('no undock', not s.undocks())
    return s


def scenario_d_mission():
    """A running mission: refused, the mission goes on."""
    s = up()
    s.run_prog('B1Test')
    check('undock', s.undocked())
    s.wait(lambda: s.busy, 20, 'busy')
    n = len(s.ev('program_active'))
    s.run_prog('AAA|||OnlyA')
    ok = s.wait(lambda: s.ss.get('state') == 'failed', 15, 'failed')
    check('refused', ok and s.ss.get('code') in ('mission_running', 'mission_waits_reset', 'not_docked', 'dock_busy'),
          str(s.ss.get('code')))
    check('no load', not s.ev('serve_site'))
    check('B still on', s.served() == 'BBB')
    check('mission untouched', len(s.ev('program_active')) == n)
    return s


def scenario_d_reset_wait():
    """Stopped mission in the dock waits for Reset: the map stays."""
    s = up()
    s.run_prog('B1Test')
    check('undock', s.undocked())
    s.wait(lambda: s.busy, 20, 'busy')
    s.stop()
    s.wait(lambda: s.status == 'Stopped', 40, 'Stopped')
    s.run_prog('AAA|||OnlyA')
    s.failed('mission_waits_reset', 20)
    check('no load', not s.ev('serve_site'))
    return s


def scenario_e_calendar():
    """Calendar slot of another map, same order, map named in the slot."""
    s = up(master_params={'idle_sleep_threshold_s': 1e7})
    ev = daily('08:00', program='Test')
    ev['site'] = 'AAA'
    start(s, D(2026, 6, 10, 7, 59, 56), ev)
    check('undock', s.undocked(90))
    s.in_order('AAA')
    check('calendar started it', [e for e in s.ev('select') if e['program'] == 'AAA|||Test'])
    check('the program of A, not of B', s.ev('program_active')[-1]['map'] == 'AAA')
    return s


def scenario_e_calendar_fail():
    """A slot whose map does not load: nothing leaves the dock, retried, reported."""
    s = up(timeout=6.0, world={'navi_block': True},
           master_params={'idle_sleep_threshold_s': 1e7, 'reject_retry_min': 0.05, 'reject_retry_max': 2})
    ev = daily('08:00', program='OnlyA')
    start(s, D(2026, 6, 10, 7, 59, 56), ev)
    ok = s.wait(lambda: any(i.get('code') == 'run_rejected' for i in s.state().get('issues', [])),
                60, 'run_rejected')
    check('reported', ok, str(s.state().get('issues')))
    issue = next((i for i in s.state().get('issues', []) if i.get('code') == 'run_rejected'), {})
    check('with the reason', 'timeout_navi' in str(issue), str(issue))
    check('no undock', not s.undocks())
    check('a second try happened', len(s.ev('select')) >= 2, str(len(s.ev('select'))))
    return s


def scenario_e_resume():
    """Resume of a program of another map keeps its checkpoint."""
    y = None
    s = Sim()
    path = os.path.join(SITES, 'AAA', 'programs.yaml')
    y = yaml.safe_load(open(path))
    y['programs'][0]['last_result'] = 'failed: on_path-Zone1-1-0'
    yaml.safe_dump(y, open(path, 'w'))
    s.boot()
    time.sleep(1.5)
    s.run_prog('AAA|||Test', resume=True)
    check('undock', s.undocked())
    s.in_order('AAA')
    act = s.ev('program_active')[-1]
    check('resume', act['resume'], str(act))
    check('checkpoint came along', act['last_result'] == 'failed: on_path-Zone1-1-0', act['last_result'])
    return s


def scenario_e_resume_across():
    """A run of A is interrupted by rain and waits, a program of B runs by hand
    meanwhile, the run resumes on A with its checkpoint, mowing where it left."""
    s = up(master_params={'idle_sleep_threshold_s': 1e7, 'weather_sleep_duration_h': 0.03,
                          'sleep_min_s': 1e7})
    ev = daily('08:00', program='OnlyA')
    ev['site'] = 'AAA'
    start(s, D(2026, 6, 10, 7, 59, 56), ev)
    check('undock', s.undocked(90))
    check('mowing on A', s.wait(lambda: len(s.mowed()) >= 1, 120, 'mowing'))
    s.set(rain_now=1, past_rain=True)
    ok = s.wait(lambda: s.stop_reason == 'interrupted:weather', 60, 'weather')
    check('rain ends it', ok, s.stop_reason)
    check('back in the dock', s.wait(lambda: len(s.undocks()) == 1 and s.status == 'Ready'
          and json.load(open(os.environ['SIM_STORE']))['s']['docked'], 60, 'dock'))
    time.sleep(1.5)
    cp = [e['last_result'] for e in s.ev('write_back') if 'on_path' in e['last_result']]
    cp = cp[-1] if cp else None
    check('checkpoint', bool(cp), str(cp))
    s.set(rain_now=0, past_rain=False)
    mark = time.time()
    s.run_prog('BBB|||B1Test')
    ok = s.wait(lambda: s.served() == 'BBB' and [e for e in s.events('program_active') if e['t'] >= mark],
                60, 'B by hand')
    check('B ran', ok)
    check('B finished', s.wait(lambda: s.stop_reason == 'succeeded' and s.status == 'Ready'
          and json.load(open(os.environ['SIM_STORE']))['s']['docked'], 180, 'B done'), s.stop_reason)
    ok = s.wait(lambda: [e for e in s.events('program_active') if e['t'] >= mark and e['map'] == 'AAA'],
                240, 'resume on A')
    check('resumed on A', ok, s.sched)
    res = [e for e in s.events('program_active') if e['t'] >= mark and e['map'] == 'AAA']
    if res:
        check('as a resume, the checkpoint kept', res[-1]['resume'] and res[-1]['last_result'] == cp,
              '%s / %s' % (res[-1], cp))
    check('A on again', s.served() == 'AAA', str(s.served()))
    y = yaml.safe_load(open(os.path.join(SITES, 'BBB', 'programs.yaml')))
    check('B keeps its result', [p for p in y['programs'] if p['name'] == 'B1Test'][0]['last_result'] == 'succeeded')
    return s


def scenario_f_legacy():
    """legacy_maps=true: load_map before the program reaches the mission."""
    s = Sim(serve='AAA')
    s.boot()
    s.kill('planner')
    # Two legacy maps made of what the planner saved for site AAA.
    run_dir = os.path.join(HOME, '.vitulus', 'running')
    saves = os.path.join(HOME, '.vitulus', 'saves', 'planner_data')
    a, b = 'LegA***env*OUTDOOR', 'LegB***env*OUTDOOR'
    for name in (a, b):
        shutil.copyfile(os.path.join(saves, 'AAA.pkl'), os.path.join(saves, name + '.pkl'))
    pkl = os.path.join(run_dir, 'planner_programs.pkl')
    import pickle
    data = pickle.load(open(pkl, 'rb'))
    lst = getattr(data, 'program_list', data)
    for p in lst:
        p.map_name = a if p.name == 'Test' else b
    pickle.dump(data, open(pkl, 'wb'), pickle.HIGHEST_PROTOCOL)
    site_file = os.path.join(run_dir, 'planner_programs_site.json')
    if os.path.exists(site_file):
        os.remove(site_file)
    rospy.set_param('/navi_manager/legacy_maps', True)
    rospy.set_param('/navi_manager/site_native', False)
    s.set(legacy=True, active=b)
    s.loaded = ''
    env = {'PYTHONPATH': os.path.join(R.PLANNER_T, 'src') + ':' + os.path.join(MAPPING_T, 'src')
                         + ':' + os.environ.get('PYTHONPATH', '')}
    s.spawn('planner', [sys.executable, os.path.join(R.PLANNER_T, 'nodes', 'node_planner'),
                        '__name:=node_planner'], env)
    check('legacy map B loaded', s.wait(lambda: s.loaded == b, 60, 'legacy load'), s.loaded)
    s.wait(lambda: s.p_sel.get_num_connections() > 0, 20, 'planner listens')
    time.sleep(1.0)
    s.mark = time.time()
    s.run_prog('Test')
    check('undock', s.undocked())
    e = s.ev()
    print('   order:', [(x['kind'], x.get('map') or x.get('action') or '') for x in e
                        if x['kind'] in ('load_map', 'navi_ready', 'program_active', 'dock_program')],
          flush=True)
    load = [x for x in s.ev('load_map') if x['map'] == a]
    active = [x for x in s.ev('program_active') if x['map'] == a]
    check('load_map of the program', bool(load))
    check('program handed over', bool(active))
    if load and active:
        check('map first, then the program, then the undock',
              load[0]['t'] < active[0]['t'] < s.undocks()[0]['t'])
    # same map again: nothing extra
    return s


def scenario_g_hold():
    """A mux hold during the load: the switch finishes, the mission does not
    start any state until the hold ends, then undocks. The hold survives it."""
    s = up(world={'navi_delay': 3.0})
    s.p_held.publish(Bool(True))
    s.run_prog('AAA|||OnlyA')
    ok = s.wait(lambda: s.ss.get('state') == 'ready', 40, 'ready')
    check('map loaded under the hold', ok and s.served() == 'AAA')
    s.still_docked(5.0)
    s.p_held.publish(Bool(False))
    check('undock after the hold', s.undocked())
    s.in_order('AAA')
    return s


def scenario_g_hold_calendar():
    """Calendar under a hold: no start at all, so no map load either."""
    s = up(master_params={'idle_sleep_threshold_s': 1e7})
    s.p_mux.publish(String('hold'))
    ev = daily('08:00', program='OnlyA')
    ev['site'] = 'AAA'
    start(s, D(2026, 6, 10, 7, 59, 56), ev)
    time.sleep(8.0)
    check('nothing selected', not s.ev('select'))
    check('no load', not s.ev('serve_site'))
    s.p_mux.publish(String('idle'))
    check('then it goes', s.undocked(90))
    s.in_order('AAA')
    return s


def scenario_h_stop():
    """Stop during the load: nothing starts."""
    s = up(world={'navi_delay': 12.0})
    s.run_prog('AAA|||OnlyA')
    ok = s.wait(lambda: s.ss.get('state') == 'switching', 15, 'switching')
    check('switching', ok, str(s.ss))
    time.sleep(1.0)
    s.stop()
    s.failed('cancelled', 15)
    s.still_docked()
    check('program never reached the mission', not s.ev('program_active'))
    return s


def scenario_i_ambiguous():
    """Same name on two other maps."""
    s = up(serve='DDD')
    s.run_prog('Test')
    s.failed('ambiguous')
    check('names the maps', 'AAA' in s.ss.get('text_cs', '') and 'BBB' in s.ss.get('text_cs', ''), s.ss.get('text_cs'))
    check('no load', not s.ev('serve_site'))
    return s


def scenario_i_race_second_run():
    """Two presses, then another program: one load, one start, one refusal."""
    s = up(world={'navi_delay': 4.0})
    s.run_prog('AAA|||OnlyA')
    time.sleep(0.5)
    s.run_prog('AAA|||OnlyA')
    time.sleep(0.5)
    s.run_prog('B1Test')
    # The refusal is followed at once by the running start again (the state topic
    # ends on that one for master_controller), so look at the history.
    refused = lambda: [x for x in s.ss_log if x.get('code') == 'switching']
    ok = s.wait(refused, 10, 'second refused')
    check('other program refused', ok, str(s.ss_log))
    check('refusal names the other program', ok and refused()[0].get('program') == 'B1Test',
          str(refused()))
    check('state ends on the running start', s.ss.get('program') == 'OnlyA', str(s.ss))
    check('undock', s.undocked())
    check('one load', len(s.ev('serve_site')) == 1)
    check('one program', len(s.ev('program_active')) == 1, str(s.ev('program_active')))
    return s


def scenario_j_edit_guard():
    """The program list after a switch is the list of the new map."""
    s = up()
    s.run_prog('AAA|||OnlyA')
    check('undock', s.undocked())
    y = yaml.safe_load(open(os.path.join(SITES, 'BBB', 'programs.yaml')))
    check('B keeps its programs', sorted(p['name'] for p in y['programs']) == ['B1Test', 'Test'],
          str([p['name'] for p in y['programs']]))
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
    return 1 if R.FAILS else 0


sys.exit(run(sys.argv[1]))
