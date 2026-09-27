#!/usr/bin/env python3
"""Regression tests for node_planner's per-site zones / programs (site-native).

No ROS master and no robot needed: rospy's publishers / subscribers / params
are stubbed and EVERYTHING runs inside a temporary $HOME (the module refuses to
run when the planner or the site-bundle library would resolve to a real one).
The ROS python libs (rospy, vitulus_msgs, vitulus_mapping) must be importable
(source the workspace); otherwise the tests are skipped.

Run:  python3 test/test_site_programs.py
"""
import copy
import importlib.machinery
import importlib.util
import json
import os
import pickle
import shutil
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_HOME = tempfile.mkdtemp(prefix='planner_test_home_')
os.environ['HOME'] = _HOME          # BEFORE the planner / bundle modules load

try:
    import yaml
    import rospy
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Point32
    from vitulus_msgs.msg import MapEditZone, PlannerProgram
    _SKIP = None
except Exception as _e:             # pragma: no cover
    _SKIP = 'ROS python libs not importable: {}'.format(_e)

M = None
EVENTS = []                         # (topic, msg) in publish order
LOGS = []                           # (level, text)
GRID = {'grid': None, 'fail': False}
STATUS = {'site': None}             # what mapping_manager's latched status says


class _Pub(object):
    def __init__(self, name, *a, **kw):
        self.name = name

    def publish(self, msg):
        EVENTS.append((self.name, msg))


class _Sub(object):
    def __init__(self, *a, **kw):
        pass

    def unregister(self):
        pass


def _wait_for_message(topic, cls, timeout=None):
    if topic == '/mapping_manager/status':
        return String(status_json(STATUS['site']))
    if GRID['fail'] or GRID['grid'] is None:
        raise rospy.ROSException('timeout')
    return GRID['grid']


def _load_module():
    global M
    rospy.Publisher = _Pub
    rospy.Subscriber = _Sub
    rospy.Service = lambda *a, **kw: None
    rospy.Timer = lambda *a, **kw: None
    rospy.get_param = lambda name, default=None: default
    rospy.get_caller_id = lambda: '/node_planner'
    rospy.wait_for_message = _wait_for_message
    rospy.Time.now = staticmethod(lambda: rospy.Time(1, 0))
    for lvl in ('loginfo', 'logwarn', 'logerr'):
        setattr(rospy, lvl, (lambda l: (
            lambda m, *a: LOGS.append((l, (m % a) if a else m))))(lvl))
    rospy.logwarn_throttle = lambda period, m, *a: LOGS.append(('logwarn', m))
    sys.path.insert(0, os.path.join(_PKG, 'src'))
    loader = importlib.machinery.SourceFileLoader(
        'node_planner_under_test', os.path.join(_PKG, 'nodes', 'node_planner'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    M = importlib.util.module_from_spec(spec)
    loader.exec_module(M)
    # SAFETY: never touch a real ~/.vitulus.
    assert M._BUNDLE_OK, 'vitulus_mapping not importable'
    assert M._VITULUS_DIR.startswith(_HOME), M._VITULUS_DIR
    assert M.vbundle.SITES_ROOT.startswith(_HOME), M.vbundle.SITES_ROOT


def grid(cells=400):
    g = OccupancyGrid()
    g.info.width = cells
    g.info.height = cells
    g.info.resolution = 0.05
    g.info.origin.orientation.w = 1.0
    g.data = [0] * (cells * cells)
    return g


def zmsg(name, x0, y0, x1, y1):
    z = MapEditZone()
    z.name = name
    z.type = 'normal'
    z.border_paths = 1
    z.paths_distance = 0.3
    z.simplify = 0.05
    z.header.frame_id = 'map'
    z.polygon.header.frame_id = 'map'
    z.polygon.polygon.points = [Point32(x0, y0, 0), Point32(x1, y0, 0),
                                Point32(x1, y1, 0), Point32(x0, y1, 0)]
    return z


def site_dir(site):
    return os.path.join(_HOME, '.vitulus', 'mapping_v3', site)


def make_site(site):
    os.makedirs(site_dir(site))
    with open(os.path.join(site_dir(site), 'manifest.yaml'), 'w') as f:
        yaml.safe_dump({'site': site, 'utm_zone': 33, 'version': 1}, f)
    write_datum(site)


def write_datum(site):
    with open(os.path.join(site_dir(site), 'datum.yaml'), 'w') as f:
        yaml.safe_dump({'utm_e': 493073.7, 'utm_n': 5540714.6, 'yaw_rad': 0.0,
                        'utm_zone': 33, 'version': 2}, f)


def yaml_programs(site):
    path = os.path.join(site_dir(site), 'programs.yaml')
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = yaml.safe_load(f)
    return {p['name']: p.get('zone_names') for p in data['programs']}


def sidecar():
    with open(M.Node._PROGRAMS_SITE_FILE) as f:
        return json.load(f)


def last(topic):
    msgs = [m for t, m in EVENTS if t == topic]
    return msgs[-1] if msgs else None


def listed_programs():
    return sorted(p.name for p in last('/web_plan/program_list').program_list)


def listed_zones():
    return sorted(z.name for z in last('/web_plan/zone_list').zone_list)


def mem_programs(n):
    return {p.name: [z.name for z in p.zone_list]
            for p in n.permanent_program_list_msg.program_list}


def new_node():
    n = M.Node()
    n._legacy_maps_cached = False       # site-native, independent of the
    n._site_native_cached = True        # robot's navi_manager.yaml
    n.callback_navi_active_map(String('SITE'))
    return n


def status_json(site):
    return json.dumps({'serving': {'site': site} if site else None})


def serve(n, site):
    STATUS['site'] = site
    n.callback_mapping_status(String(status_json(site)))


def program(name, zones=()):
    p = PlannerProgram()
    p.name = name
    p.map_name = 'SITE'
    p.speed = 'mid'
    p.zone_list = [copy.deepcopy(z) for z in zones]
    return p


def zone_msgs(n, *names):
    by_name = {z.msg.name: z.msg for z in n.map_data.zones}
    return [by_name[x] for x in names]


def forget_workspace_pickle(site):
    """Force the 'no saved pickle -> regenerate from the bundle' path."""
    if os.path.exists(M._saves_pkl(site)):
        os.remove(M._saves_pkl(site))


@unittest.skipIf(_SKIP is not None, _SKIP)
class SiteProgramsTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _load_module()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_HOME, ignore_errors=True)

    def setUp(self):
        shutil.rmtree(os.path.join(_HOME, '.vitulus'), ignore_errors=True)
        for site in ('A', 'B'):
            make_site(site)
        del EVENTS[:]
        del LOGS[:]
        GRID['grid'] = grid(400)
        GRID['fail'] = False
        STATUS['site'] = None

    def _site_a_with_mowall(self):
        """Site A: zones Near + Far, program MowAll[Near, Far]; owner = A."""
        n = new_node()
        serve(n, 'A')
        n.callback_map_save_zone(zmsg('Near', 2, 2, 6, 6))
        n.callback_map_save_zone(zmsg('Far', 12, 12, 17, 17))
        n.callback_program_new(program('MowAll', zone_msgs(n, 'Near', 'Far')))
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        return n

    # ------------------------------------------------------------ normal path
    def test_switch_a_b_none_a(self):
        n = self._site_a_with_mowall()
        n.callback_program_to_show_marker(program('MowAll', zone_msgs(n, 'Near')))
        self.assertEqual(sidecar(), {'site': 'A'})

        del EVENTS[:]
        serve(n, 'B')
        marker_msgs = [m for t, m in EVENTS if t == '/web_plan/program_marker']
        self.assertEqual([[k.action for k in m.markers] for m in marker_msgs], [[3]])
        self.assertIsNone(n.current_program_marker_array)
        self.assertEqual([k.action for k in last('/web_plan/zone_preview_marker').markers], [3])
        self.assertEqual(listed_zones(), [])
        self.assertEqual(listed_programs(), [])
        self.assertEqual(sidecar(), {'site': 'B'})
        n.callback_map_save_zone(zmsg('OnB', 1, 1, 5, 5))
        n.callback_program_new(program('P_B', zone_msgs(n, 'OnB')))

        serve(n, None)                      # serving stopped: nothing is lost
        self.assertEqual(mem_programs(n), {'P_B': ['OnB']})
        self.assertEqual(sidecar(), {'site': 'B'})

        serve(n, 'A')
        self.assertEqual(listed_zones(), ['Far', 'Near'])
        self.assertEqual(listed_programs(), ['MowAll'])
        self.assertEqual(mem_programs(n), {'MowAll': ['Near', 'Far']})
        self.assertTrue(all(len(z.paths) > 0 for z in
                            n.permanent_program_list_msg.program_list[0].zone_list))
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        self.assertEqual(yaml_programs('B'), {'P_B': ['OnB']})
        self.assertEqual(sidecar(), {'site': 'A'})

    def test_restart_and_switch_back_use_pickle_no_regen(self):
        """AUDIT A4: zones come back from saves/<site>.pkl, not from a full
        regeneration, on a node start and on a switch back."""
        n = self._site_a_with_mowall()
        serve(n, 'B')
        calls = []
        orig = M.Node._regenerate_zone
        M.Node._regenerate_zone = lambda self, msg: calls.append(msg.name) or orig(self, msg)
        try:
            serve(n, 'A')
            n2 = new_node()
            serve(n2, 'A')
        finally:
            M.Node._regenerate_zone = orig
        self.assertEqual(calls, [])
        self.assertEqual(sorted(z.msg.name for z in n2.map_data.zones), ['Far', 'Near'])
        self.assertEqual(mem_programs(n2), {'MowAll': ['Near', 'Far']})

    # -------------------------------------------------------------- finding 1
    def test_f1_raster_failure_never_empties_programs(self):
        n = self._site_a_with_mowall()
        serve(n, 'B')
        forget_workspace_pickle('A')
        stamp_before = n._bundle_stamp_get('A', 'programs')
        GRID['fail'] = True                 # site_map not delivered in time
        serve(n, 'A')
        self.assertIsNone(n.map_data)
        self.assertEqual(sidecar(), {'site': 'B'})          # NOT swapped
        self.assertEqual(n._bundle_stamp_get('A', 'programs'), stamp_before)
        self.assertEqual(listed_programs(), [])             # B's list is hidden on A
        # mission SM / UI posts a program while the swap is pending: refused
        n.callback_program_new(program('P2'))
        n.callback_program_remove(String('MowAll'))
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        self.assertEqual(yaml_programs('B'), None)
        # the late site_map arrives -> retry completes everything
        GRID['fail'] = False
        n._do_live_refresh(GRID['grid'])
        self.assertEqual(sidecar(), {'site': 'A'})
        self.assertEqual(mem_programs(n), {'MowAll': ['Near', 'Far']})
        self.assertEqual(listed_programs(), ['MowAll'])
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})

    def test_f1_missing_datum_never_empties_programs(self):
        n = self._site_a_with_mowall()
        serve(n, 'B')
        forget_workspace_pickle('A')
        os.remove(os.path.join(site_dir('A'), 'datum.yaml'))
        serve(n, 'A')                       # raster OK, zones cannot be imported
        self.assertEqual(sidecar(), {'site': 'B'})
        self.assertEqual(listed_programs(), [])
        n.callback_program_new(program('P2'))
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        write_datum('A')
        n.callback_reload(String(''))       # UI reload = retry
        self.assertEqual(mem_programs(n), {'MowAll': ['Near', 'Far']})
        self.assertEqual(sidecar(), {'site': 'A'})

    def test_f1_zone_outside_raster_keeps_yaml_zone_names(self):
        n = self._site_a_with_mowall()
        serve(n, 'B')
        forget_workspace_pickle('A')
        # the served raster no longer covers 'Far': its regeneration fails
        orig = M.Node._regenerate_zone

        def regen(self, msg):
            if msg.name == 'Far':
                raise ValueError('zone outside the raster')
            return orig(self, msg)
        M.Node._regenerate_zone = regen
        try:
            serve(n, 'A')
            self.assertEqual(sorted(z.msg.name for z in n.map_data.zones), ['Near'])
            self.assertEqual(mem_programs(n), {'MowAll': ['Near']})
            n.callback_program_new(program('Other'))        # any export
            n._do_live_refresh(grid(300))                    # ... and another
        finally:
            M.Node._regenerate_zone = orig
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far'], 'Other': []})
        # full coverage again -> the program is whole again
        serve(n, 'B')
        forget_workspace_pickle('A')
        serve(n, 'A')
        self.assertEqual(mem_programs(n)['MowAll'], ['Near', 'Far'])

    def test_removed_zone_leaves_the_program_yaml(self):
        n = self._site_a_with_mowall()
        n.callback_zone_remove(String('Far'))
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near']})

    # ------------------------------------------ review 2: site-change races
    def _zones_geojson(self, site):
        path = os.path.join(site_dir(site), 'zones.geojson')
        if not os.path.exists(path):
            return None
        return sorted(z.get('name') for z in M.vbundle.load_zones(site))

    def _pickled_zones(self, site):
        with open(M._saves_pkl(site), 'rb') as f:
            md = pickle.load(f)
        return (sorted((z.msg.name, z.msg.area, len(z.msg.paths)) for z in md.zones),
                md.initial_map.info.width)

    def test_r1_zones_never_exported_into_another_site(self):
        """The served site flips to B while a zone save of A is in flight."""
        n = self._site_a_with_mowall()
        orig = n._propagate_zone_to_programs

        def flip(zone_msg):
            n._served_site = 'B'            # what a status message in flight did
            return orig(zone_msg)
        n._propagate_zone_to_programs = flip
        n.callback_map_save_zone(zmsg('Near', 2, 2, 7, 7))
        n.callback_zone_remove(String('Near'))
        self.assertIsNone(self._zones_geojson('B'))
        self.assertFalse(os.path.exists(M._saves_pkl('B')))
        self.assertEqual(yaml_programs('B'), None)
        n._served_site = 'A'
        n._propagate_zone_to_programs = orig
        serve(n, 'B')
        self.assertEqual([z.msg.name for z in n.map_data.zones], [])

    def test_r1_served_site_changes_under_the_workspace_lock(self):
        import threading
        n = self._site_a_with_mowall()
        done = threading.Event()
        with n._workspace_lock:             # a save / refresh of A is running
            t = threading.Thread(target=lambda: (serve(n, 'B'), done.set()))
            t.daemon = True
            t.start()
            self.assertFalse(done.wait(0.3))
            self.assertEqual(n._served_site, 'A')
        self.assertTrue(done.wait(30))
        self.assertEqual(n._served_site, 'B')

    def test_r2_refresh_before_status_never_touches_the_old_site(self):
        """B's site_map is processed while A still counts as served."""
        n = self._site_a_with_mowall()
        before = self._pickled_zones('A')
        geojson_before = open(os.path.join(site_dir('A'), 'zones.geojson')).read()
        areas = {p.name: p.area for p in n.permanent_program_list_msg.program_list}
        STATUS['site'] = 'B'                # mapping_manager already serves B ...
        n._do_live_refresh(grid(200))       # ... the planner's status cb did not run yet
        self.assertEqual(self._pickled_zones('A'), before)
        self.assertEqual(open(os.path.join(site_dir('A'), 'zones.geojson')).read(),
                         geojson_before)
        self.assertEqual({p.name: p.area for p in n.permanent_program_list_msg.program_list},
                         areas)
        self.assertEqual(n.map_data.initial_map.info.width, 400)
        # status unavailable -> cannot tell -> skipped as well
        orig = rospy.wait_for_message

        def no_status(topic, cls, timeout=None):
            if topic == '/mapping_manager/status':
                raise rospy.ROSException('timeout')
            return orig(topic, cls, timeout)
        rospy.wait_for_message = no_status
        try:
            n._do_live_refresh(grid(200))
        finally:
            rospy.wait_for_message = orig
        self.assertEqual(self._pickled_zones('A'), before)
        # the normal refresh (same site) still works
        STATUS['site'] = 'A'
        n._do_live_refresh(grid(300))
        self.assertEqual(self._pickled_zones('A')[1], 300)

    def test_r3_crash_in_first_swap_keeps_pickle_only_program(self):
        self._pickle_with('Orphan')          # owner unknown, only in the pickle
        M.planner_io.write_json_atomic(M.Node._PROGRAMS_SITE_FILE, {
            'site': None, 'transition': {'from': None, 'to': 'A'}})
        n = new_node()
        serve(n, 'A')
        self.assertEqual(mem_programs(n), {'Orphan': []})
        self.assertEqual(yaml_programs('A'), {'Orphan': []})

    def test_r4_pending_state_is_reported_once(self):
        n = self._site_a_with_mowall()
        serve(n, 'B')
        forget_workspace_pickle('A')
        GRID['fail'] = True
        del EVENTS[:]
        del LOGS[:]
        serve(n, 'A')
        for _ in range(3):                   # master_controller keeps retrying
            n.callback_program_select_resume(String('MowAll'))
            n.callback_program_select(String('MowAll'))
            n.callback_reload(String(''))
        logs = [m.data for t, m in EVENTS if t == '/web_plan/log']
        state = "Programs not loaded: zones or datum of map 'A' are not ready"
        self.assertEqual(logs.count(state), 1)
        self.assertEqual(len([x for x in logs if x.startswith(state + ' — ')]), 1)
        self.assertFalse(any('not found' in x for x in logs))
        self.assertEqual(len([1 for l, m in LOGS if l == 'logerr' and state in m]), 1)
        self.assertEqual(len([1 for t, m in EVENTS if t == '/web_plan/program_active']), 0)
        GRID['fail'] = False
        n.callback_reload(String(''))
        n.callback_program_select(String('MowAll'))
        self.assertEqual(len([1 for t, m in EVENTS if t == '/web_plan/program_active']), 1)

    # -------------------------------------------------------------- finding 2
    def _pickle_with(self, *names):
        n = new_node()
        for name in names:
            n.permanent_program_list_msg.program_list.append(program(name))
        n.save_programs()
        if os.path.exists(M.Node._PROGRAMS_SITE_FILE):
            os.remove(M.Node._PROGRAMS_SITE_FILE)

    def test_f2_adopted_program_is_exported_and_survives_switches(self):
        self._pickle_with('Orphan')          # pre-sidecar data: owner unknown
        n = new_node()
        serve(n, 'A')
        self.assertEqual(mem_programs(n), {'Orphan': []})
        self.assertEqual(yaml_programs('A'), {'Orphan': []})   # exported at once
        serve(n, 'B')
        self.assertEqual(mem_programs(n), {})
        serve(n, 'A')
        self.assertEqual(mem_programs(n), {'Orphan': []})
        self.assertEqual(yaml_programs('B'), None)

    def test_f2_program_created_with_nothing_served_survives_swap(self):
        n = new_node()
        serve(n, 'A')
        serve(n, None)
        n.callback_program_new(program('Offline'))           # export impossible
        self.assertEqual(yaml_programs('A'), None)
        serve(n, 'B')                                         # pre-swap save to A
        self.assertEqual(yaml_programs('A'), {'Offline': []})
        serve(n, 'A')
        self.assertEqual(mem_programs(n), {'Offline': []})

    def test_f2_swap_refused_when_old_list_cannot_be_saved(self):
        n = self._site_a_with_mowall()
        os.remove(os.path.join(site_dir('A'), 'programs.yaml'))   # only in memory now
        orig = M.vbundle.save_programs

        def failing(site, progs):
            raise IOError('disk full')
        M.vbundle.save_programs = failing
        try:
            serve(n, 'B')
        finally:
            M.vbundle.save_programs = orig
        self.assertEqual(sidecar(), {'site': 'A'})
        self.assertEqual(mem_programs(n), {'MowAll': ['Near', 'Far']})
        self.assertEqual(listed_programs(), [])              # hidden on B
        n.callback_reload(String(''))                         # retry
        self.assertEqual(sidecar(), {'site': 'B'})
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})

    def test_f2_pickle_and_back_are_different_generations(self):
        n = self._site_a_with_mowall()
        serve(n, 'B')                        # pickle now holds B's (empty) list
        with open(M._PROGRAMS_PKL + '.BACK', 'rb') as f:
            back = pickle.load(f)
        self.assertEqual([p.name for p in back.program_list], ['MowAll'])

    # -------------------------------------------------------------- finding 4
    def test_f4_remove_unknown_program_never_wipes_yaml(self):
        n = self._site_a_with_mowall()
        for path in (M._PROGRAMS_PKL, M._PROGRAMS_PKL + '.BACK'):
            with open(path, 'wb') as f:
                f.write(b'corrupt')
        n2 = new_node()
        self.assertEqual(mem_programs(n2), {})
        n2.callback_program_remove(String('NoSuchProgram'))   # nothing served
        serve(n2, 'A')
        n2.callback_program_remove(String('NoSuchProgram'))   # served
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        # ... and the unreadable pickle was replaced by the yaml's list
        self.assertEqual(mem_programs(n2), {'MowAll': ['Near', 'Far']})
        n2.callback_program_remove(String('MowAll'))          # a REAL removal
        self.assertEqual(yaml_programs('A'), {})

    # -------------------------------------------------------------- finding 5
    def test_f5_interrupted_swap_rebuilds_from_bundle(self):
        n = self._site_a_with_mowall()
        # crash between the transition marker and the new pickle
        M.planner_io.write_json_atomic(M.Node._PROGRAMS_SITE_FILE, {
            'site': None, 'transition': {'from': 'A', 'to': 'B'}})
        n2 = new_node()
        serve(n2, 'B')
        self.assertEqual(mem_programs(n2), {})               # A's list not adopted by B
        self.assertEqual(yaml_programs('B'), None)
        self.assertEqual(yaml_programs('A'), {'MowAll': ['Near', 'Far']})
        self.assertEqual(sidecar(), {'site': 'B'})

    # -------------------------------------------------------------- finding 6
    def _zone_results(self):
        return [json.loads(m.data) for t, m in EVENTS if t == '/web_plan/zone_result']

    def test_f6_exactly_one_zone_result_per_attempt(self):
        n = new_node()
        n.callback_map_save_zone(zmsg('NoMap', 2, 2, 6, 6))   # nothing served
        serve(n, 'A')
        n.callback_map_save_zone(zmsg('Tiny', 2, 2, 2.1, 2.1))
        del EVENTS[:]
        n.callback_map_save_zone(zmsg('Good', 2, 2, 6, 6))
        topics = [t for t, m in EVENTS]
        self.assertLess(topics.index('/web_plan/zone_list'),
                        topics.index('/web_plan/zone_result'))
        orig = M.Node._bundle_export_zones

        def boom(self):
            raise RuntimeError('unexpected')
        M.Node._bundle_export_zones = boom
        try:
            n.callback_map_save_zone(zmsg('Boom', 8, 8, 12, 12))
            n.callback_zone_remove(String('Good'))
        finally:
            M.Node._bundle_export_zones = orig
        n.callback_zone_remove(String('Boom'))
        got = [(r['op'], r['name'], r['ok']) for r in self._zone_results()]
        self.assertEqual(got, [('save', 'Good', True), ('save', 'Boom', False),
                               ('remove', 'Good', False), ('remove', 'Boom', True)])
        all_results = [t for t, m in EVENTS if t == '/web_plan/zone_result']
        self.assertEqual(len(all_results), 4)

    def test_f6_failures_before_a_map_exists(self):
        n = new_node()
        n.callback_map_save_zone(zmsg('NoMap', 2, 2, 6, 6))
        n.callback_zone_remove(String('NoMap'))
        n.callback_zone_selected(String('NoMap'))
        got = [(r['op'], r['ok']) for r in self._zone_results()]
        self.assertEqual(got, [('save', False), ('remove', False)])
        self.assertIn('no map is active', self._zone_results()[0]['message'])

    # ------------------------------------------ vitulus-field#1 S19: no serving
    def test_s19_save_refused_after_serving_deactivated(self):
        n = self._site_a_with_mowall()
        geojson_before = self._zones_geojson('A')
        serve(n, None)                      # UI "Deactivate"
        self.assertIsNone(n.map_data)
        self.assertEqual(listed_zones(), [])
        n._do_live_refresh(grid(1))         # mapping_manager's 1x1 placeholder
        self.assertIsNone(n.map_data)
        del EVENTS[:]
        n.callback_map_save_zone(zmsg('S19test', 2, 2, 6, 6))
        n.callback_zone_remove(String('Near'))
        got = [(r['op'], r['ok'], r['message']) for r in self._zone_results()]
        self.assertEqual(got, [
            ('save', False, "Zone 'S19test' NOT saved: no map is active — "
                            "activate or record a map first."),
            ('remove', False, "Zone 'Near' NOT removed: no map is active.")])
        self.assertNotIn('/web_plan/zone_list', [t for t, m in EVENTS])
        serve(n, 'A')                       # the site's zones come back intact
        self.assertEqual(listed_zones(), ['Far', 'Near'])
        self.assertEqual(self._zones_geojson('A'), geojson_before)
        self.assertEqual(mem_programs(n), {'MowAll': ['Near', 'Far']})

    def test_s19_workspace_without_served_site_is_not_editable(self):
        """A workspace in memory while nothing is served belongs to no bundle."""
        self._site_a_with_mowall()
        n = new_node()
        shutil.copyfile(M._saves_pkl('A'), M._RUNNING_PKL)
        n.data_loaded = n.load_running_data()
        self.assertIsNotNone(n.map_data)
        before = self._pickled_zones('A')
        n._do_live_refresh(grid(1))
        self.assertEqual(self._pickled_zones('A'), before)
        del EVENTS[:]
        n.callback_map_save_zone(zmsg('Lost', 2, 2, 6, 6))
        self.assertEqual([(r['op'], r['ok']) for r in self._zone_results()],
                         [('save', False)])
        self.assertEqual(self._zones_geojson('A'), ['Far', 'Near'])

    # -------------------------------------------------------------- finding 7
    def test_f7_select_of_unknown_program_is_reported(self):
        n = self._site_a_with_mowall()
        del EVENTS[:]
        n.callback_program_select(String('MowAll'))
        n.callback_program_select(String('Elsewhere'))
        logs = [m.data for t, m in EVENTS if t == '/web_plan/log']
        self.assertIn("Program 'Elsewhere' not found on the active map", logs)
        self.assertEqual(len([1 for t, m in EVENTS if t == '/web_plan/program_active']), 1)


if __name__ == '__main__':
    unittest.main()
