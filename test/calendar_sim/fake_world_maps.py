#!/usr/bin/env python3
"""fake_world for run_program_map.py: the REAL planner and the REAL
mapping_manager run beside it, so its fake planner stays silent, and it fakes
navi_man instead: active_map, /navi_manager/map_ready with a delay, and the dock
hold that takes a moment to place the robot on the new map's dock.

/sim/set keys: navi_delay (s), navi_block (never takes the map over),
dock_err (m, final), dock_err_first (m, for the first second), legacy (load_map),
has_dock.
"""
import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from vitulus_msgs.msg import PlannerProgram

SILENT = ('/mapping_manager/status', '/web_plan/planner_loaded', '/web_plan/program_list',
          '/web_plan/program_active', '/web_plan/program_active_unfinished')
_publisher = rospy.Publisher


class _Null(object):
    def publish(self, *_a, **_k):
        pass

    def get_num_connections(self):
        return 0


def _pub(name, *a, **k):
    return _Null() if name in SILENT else _publisher(name, *a, **k)


rospy.Publisher = _pub
import fake_world as W   # noqa: E402


class World(W.World):
    def __init__(self):
        W.World.__init__(self)
        self.s.setdefault('navi_delay', 1.0)
        self.s.setdefault('navi_block', False)
        self.s.setdefault('dock_err', 0.02)
        self.s.setdefault('dock_err_first', 4.2)
        self.s.setdefault('has_dock', True)
        self.s.setdefault('legacy', False)
        self.served = None
        self.points_site = None
        self.took_over = 0.0
        self.active = 'SITE'
        self.loading = False
        self.p_active_map = _publisher('/navi_manager/active_map', String, queue_size=1)
        self.p_ready = _publisher('/navi_manager/map_ready', String, queue_size=1, latch=True)
        self.p_docked = _publisher('/dock_manager/is_in_dock', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/mapping_manager/status', String, self.on_status)
        rospy.Subscriber('/mapping_manager/serve_site', String,
                         lambda m: W.rec('serve_site', site=m.data))
        rospy.Subscriber('/navi_manager/load_map', String, self.on_load_map)
        rospy.Subscriber('/navi_manager/load_map_rtabmap', String, self.on_load_map)
        rospy.Subscriber('/web_plan/program_active', PlannerProgram, self.on_active, (False,))
        rospy.Subscriber('/web_plan/program_active_unfinished', PlannerProgram, self.on_active, (True,))

    def on_select(self, msg, args):
        W.rec('select', program=msg.data, resume=args[0])

    def on_write_back(self, msg):
        W.rec('write_back', last_result=msg.last_result)

    def publish_list(self):
        pass

    def on_active(self, msg, args):
        W.rec('program_active', program=msg.name, map=msg.map_name, resume=args[0],
              last_result=msg.last_result)

    def on_status(self, msg):
        serving = json.loads(msg.data).get('serving') or {}
        site = serving.get('site')
        if site == self.served:
            return
        self.served = site
        W.rec('serving', site=site)
        threading.Thread(target=self.take_over, args=(site,), daemon=True).start()

    def take_over(self, site):
        time.sleep(float(self.s['navi_delay']))
        if self.s['navi_block'] or site != self.served:
            return
        self.points_site = site
        self.took_over = time.time()
        W.rec('navi_ready', site=site)

    def on_load_map(self, msg):
        W.rec('load_map', map=msg.data)
        self.loading = True
        self.active = msg.data
        threading.Thread(target=self.legacy_loaded, daemon=True).start()

    def legacy_loaded(self):
        time.sleep(float(self.s['navi_delay']))
        if not self.s['navi_block']:
            self.loading = False
            W.rec('navi_ready', map=self.active)

    def on_set(self, msg):
        W.World.on_set(self, msg)
        d = json.loads(msg.data)
        if d.get('active'):
            self.active = d['active']

    def tick(self):
        W.World.tick(self)
        self.p_docked.publish(Bool(self.s['docked']))
        self.p_active_map.publish(String(self.active))
        fresh = time.time() - self.took_over < 1.0
        err = float(self.s['dock_err_first'] if fresh else self.s['dock_err'])
        legacy = bool(self.s['legacy'])
        ready = (not self.loading) if legacy else (
            bool(self.served) and self.points_site == self.served)
        self.p_ready.publish(String(json.dumps({
            'active_map': self.active, 'site': self.points_site, 'served': self.served,
            'ready': ready, 'has_dock': bool(self.s['has_dock']),
            'has_docked': True, 'dock_err_m': err if ready else None})))


if __name__ == '__main__':
    rospy.init_node('fake_world')
    w = World()
    W.rec('world_up')
    r = rospy.Rate(20)
    n = 0
    while not rospy.is_shutdown():
        w.tick()
        n += 1
        if n % 10 == 0:
            w.slow()
        if n % 20 == 0:
            w.store()
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            break
