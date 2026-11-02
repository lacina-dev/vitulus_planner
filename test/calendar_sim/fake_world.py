#!/usr/bin/env python3
"""Everything around the real mower_unit_smach + master_controller, for the
calendar simulation (run_sim.py). PRIVATE roscore only.

Fakes: power module, dock + dock_smach, planner (program list, select, the
mission's write-back, persisted like the real one), mower, MBF (get_path,
exe_path that really moves a point along the path), TF, GNSS status, weather.

Control: /sim/set (JSON String): battery, rain_now, past_rain, in_dock, ...
Record:  $SIM_LOG, one JSON per line: exe chunks, selects, dock actions.
"""
import json
import math
import os
import threading
import time

import actionlib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathResult,
                          GetPathAction, GetPathResult, MoveBaseAction,
                          MoveBaseResult, RecoveryAction, RecoveryResult)
from mbf_msgs.srv import CheckPath, CheckPathResponse
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int8, Int16, String, UInt64
from vitulus_msgs.msg import (DockProgram, MapEditZone, Mower, Navi_transform,
                              PlannerProgram, PlannerProgramList, Power_status,
                              StringList)
from weather_alert.msg import RainAlert

SIM_LOG = os.environ['SIM_LOG']
STORE = os.environ['SIM_STORE']        # planner + world state across restarts
PATH_LEN = float(os.environ.get('SIM_PATH_LEN', '10'))
SPEED = float(os.environ.get('SIM_SPEED', '2'))


def rec(kind, **kw):
    kw.update(kind=kind, t=time.time())
    with open(SIM_LOG, 'a') as f:
        f.write(json.dumps(kw) + '\n')


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p


def line(y, n=30, reverse=False):
    path = Path()
    path.header.frame_id = 'map'
    for i in range(n):
        f = i / (n - 1.0)
        path.poses.append(pose(PATH_LEN * (1.0 - f if reverse else f), y))
    return path


def make_zone(name, y0):
    z = MapEditZone()
    z.name = name
    z.paths = [line(y0 + i, reverse=bool(i % 2)) for i in range(3)]
    z.cut_height = 40
    z.rpm = 2500
    return z


class World(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.s = {'battery': 90.0, 'rain_now': 0, 'past_rain': False,
                  'forecast_rain': False, 'x': -3.0, 'y': 0.0, 'docked': True,
                  'no_weather': False, 'stale_weather': False, 'undock_fail': False,
                  'charge_rate': 1.0, 'drain_rate': 0.0, 'smach_lock': False,
                  'mower_silent': False, 'mower_heal': '', 'blade_dead': False,
                  'mcu_dead': False}
        self.prog = {'name': 'Mow', 'last_result': '', 'last_duration_minutes': 0}
        if os.path.exists(STORE):
            d = json.load(open(STORE))
            self.s.update(d['s'])
            self.prog.update(d['prog'])
        self.mower = {'power': False, 'motor': False, 'rpm': 0, 'height': 60}
        self.bat_out = True
        self.holds = set()          # /cmd_vel_mux/hold
        self.dock_status = 0 if self.s['docked'] else 3
        self.dock_busy = 0.0
        self.motors = False
        self.zones = [make_zone('ZoneA', 0.0), make_zone('ZoneB', 10.0)]

        self.p_pm = rospy.Publisher('/pm/power_status', Power_status, queue_size=1)
        # read-back of the sleep commands, as /pm/power_status does
        self.pm_sleep = {'sleep_status': False, 'sleep_charged_status': False,
                         'sleep_wait_standby': 15000, 'sleep_time_interval': 0,
                         'sleep_wait_charged': 120000}
        self.pm_sleep.update(self.s.get('pm_eeprom') or {})     # survives the cut
        for topic, kind, field in (
                ('set_robot_sleep', Bool, 'sleep_status'),
                ('set_sleep_until_charged', Bool, 'sleep_charged_status'),
                ('set_sleep_time', UInt64, 'sleep_time_interval'),
                ('set_sleep_wait_before_standby', UInt64, 'sleep_wait_standby'),
                ('set_sleep_wait_charged_offset', UInt64, 'sleep_wait_charged')):
            rospy.Subscriber('/pm/' + topic, kind, self.pm_sleep_cmd, field)
        self.p_in_dock = rospy.Publisher('/dock_manager/is_in_dock_confirmed', Bool,
                                         queue_size=1, latch=True)
        self.p_dock = rospy.Publisher('/dock_smach/dock_status', Int8, queue_size=1, latch=True)
        self.p_undock_prog = rospy.Publisher('/dock_manager/undock_program', DockProgram,
                                             queue_size=1, latch=True)
        self.p_dock_prog = rospy.Publisher('/dock_manager/dock_program', DockProgram,
                                           queue_size=1, latch=True)
        self.p_points = rospy.Publisher('/navi_manager/map_point_str_list', StringList,
                                        queue_size=1)
        self.p_odom = rospy.Publisher('/nav_tf/odom_status', Navi_transform, queue_size=1)
        self.p_site = rospy.Publisher('/mapping_manager/status', String, queue_size=1, latch=True)
        self.p_loaded = rospy.Publisher('/web_plan/planner_loaded', String, queue_size=1, latch=True)
        self.p_list = rospy.Publisher('/web_plan/program_list', PlannerProgramList,
                                      queue_size=1, latch=True)
        self.p_active = rospy.Publisher('/web_plan/program_active', PlannerProgram, queue_size=1)
        self.p_unfinished = rospy.Publisher('/web_plan/program_active_unfinished',
                                            PlannerProgram, queue_size=1)
        self.p_mower = rospy.Publisher('/mower/status', Mower, queue_size=1)
        self.p_link = rospy.Publisher('/mower/link', String, queue_size=1)
        self.silent_t = None
        self.p_motor_state = rospy.Publisher('/base/motor_power_state', Bool, queue_size=1)
        self.p_rain = rospy.Publisher('/weather_alert/rain_alert', RainAlert, queue_size=1)
        self.p_held = rospy.Publisher('/cmd_vel_mux/held', Bool, queue_size=1, latch=True)
        self.tf = tf.TransformBroadcaster()

        rospy.Subscriber('/sim/set', String, self.on_set)
        rospy.Subscriber('/dock_smach/start_docking', DockProgram, self.on_dock_program)
        rospy.Subscriber('/dock_smach/stop', Bool, self.on_dock_stop)
        rospy.Subscriber('/web_plan/program_select', String, self.on_select, (False,))
        rospy.Subscriber('/web_plan/program_select_resume', String, self.on_select, (True,))
        rospy.Subscriber('/web_plan/program_new', PlannerProgram, self.on_write_back)
        rospy.Subscriber('/mower/set_power', Bool, self.on_power)
        rospy.Subscriber('/mower/set_motor_on', Bool, self.on_motor)
        rospy.Subscriber('/mower/set_motor_rpm', Int16, self.on_rpm)
        rospy.Subscriber('/mower/set_cut_height', Int16, self.on_height)
        rospy.Subscriber('/mower/link_cmd', String, self.on_link_cmd)
        rospy.Subscriber('/pm/set_bat_out_switch', Bool, self.on_bat_out)
        rospy.Subscriber('/cmd_vel_mux/hold', String, self.on_hold)

        self.a_mb = actionlib.SimpleActionServer('/move_base_flex/move_base', MoveBaseAction,
                                                 lambda g: self.a_mb.set_succeeded(MoveBaseResult()),
                                                 auto_start=False)
        self.a_get = actionlib.SimpleActionServer('/move_base_flex/get_path', GetPathAction,
                                                  self.do_get_path, auto_start=False)
        self.a_exe = actionlib.SimpleActionServer('/move_base_flex/exe_path', ExePathAction,
                                                  self.do_exe_path, auto_start=False)
        self.a_rec = actionlib.SimpleActionServer('/move_base_flex/recovery', RecoveryAction,
                                                  lambda g: self.a_rec.set_succeeded(RecoveryResult()),
                                                  auto_start=False)
        for a in (self.a_mb, self.a_get, self.a_exe, self.a_rec):
            a.start()
        rospy.Service('/move_base_flex/check_path_cost', CheckPath,
                      lambda r: CheckPathResponse(last_checked=0, state=0, cost=0))

        dp = DockProgram()
        dp.action = 'UNDOCK'
        dp.path = 'UNDOCK'
        self.p_undock_prog.publish(dp)
        dp2 = DockProgram()
        dp2.action = 'DOCK'
        dp2.path = 'DOCK'
        self.p_dock_prog.publish(dp2)
        self.p_site.publish(String(json.dumps({'serving': {'site': 'sim'}})))
        self.p_loaded.publish(String('sim'))
        self.publish_list()

    # --- persistence --------------------------------------------------------
    def store(self):
        tmp = '%s.%d.tmp' % (STORE, threading.get_ident())
        with open(tmp, 'w') as f:
            json.dump({'s': self.s, 'prog': self.prog}, f)
        os.replace(tmp, STORE)

    def on_set(self, msg):
        with self.lock:
            d = json.loads(msg.data)
            self.s.update(d)
            if d.get('docked') is True:
                self.dock_status = 0
                self.s.update(x=-3.0, y=0.0)
            if d.get('docked') is False:
                self.dock_status = 3
            self.store()

    # --- planner ------------------------------------------------------------
    def program(self):
        p = PlannerProgram()
        p.header.frame_id = 'map'
        p.name = self.prog['name']
        p.map_name = 'SITE'
        p.last_result = self.prog['last_result']
        p.zone_list = list(self.zones)
        p.speed = 'mid'
        return p

    def publish_list(self):
        m = PlannerProgramList()
        m.program_list = [self.program()]
        self.p_list.publish(m)

    def on_select(self, msg, args):
        resume = args[0]
        rec('select', program=msg.data, resume=resume, last_result=self.prog['last_result'])
        (self.p_unfinished if resume else self.p_active).publish(self.program())

    def on_write_back(self, msg):
        if msg.last_result != self.prog['last_result']:
            rec('write_back', last_result=msg.last_result)
        self.prog['last_result'] = msg.last_result
        self.store()
        self.publish_list()

    # --- mower --------------------------------------------------------------
    def silent(self):
        """A dead link: nothing reaches the board, the blade stands."""
        return self.s['mower_silent']

    def on_power(self, msg):
        rec('mower_power', on=msg.data, silent=self.silent())
        if not self.silent():
            self.mower['power'] = msg.data
            if msg.data:
                self.bat_out = True     # the firmware asks the PM for it

    def on_motor(self, msg):
        rec('mower_motor', on=msg.data, silent=self.silent())
        if not self.silent():
            self.mower['motor'] = msg.data
            if msg.data:
                self.s['blade_dead'] = False

    def on_rpm(self, msg):
        if not self.silent():
            self.mower['rpm'] = msg.data

    def on_height(self, msg):
        if not self.silent():
            self.mower['height'] = msg.data

    def heal(self):
        """The board restarted: at max height, blade off, rpm from its EEPROM."""
        self.s['mower_silent'] = False
        self.mower.update(motor=False, power=True, height=60, rpm=2000)

    def on_link_cmd(self, msg):
        rec('link_cmd', cmd=msg.data)
        if msg.data == 'restart_serial' and self.s['mower_heal'] == 'serial':
            self.heal()

    def on_bat_out(self, msg):
        rec('bat_out', on=msg.data, silent=self.silent())
        self.bat_out = msg.data
        if not msg.data:
            self.mower['motor'] = False
            if self.s['mower_heal'] == 'power':     # it answers with the rail off
                self.heal()
                self.mower['power'] = False

    def on_hold(self, msg):
        req = json.loads(msg.data)
        before = bool(self.holds)
        (self.holds.add if req['hold'] else self.holds.discard)(req['source'])
        if bool(self.holds) != before:
            rec('mux_hold', on=bool(self.holds), x=round(self.s['x'], 3), y=round(self.s['y'], 3))
        self.p_held.publish(Bool(bool(self.holds)))

    # --- dock ---------------------------------------------------------------
    def on_dock_program(self, msg):
        rec('dock_program', action=msg.action)
        threading.Thread(target=self.dock_action, args=(msg.action,), daemon=True).start()

    def dock_action(self, action):
        if action == 'UNDOCK':
            self.dock_status = 1
            self.motors = True
            time.sleep(2.0)
            if self.dock_status != 1:
                return
            if self.s['undock_fail']:
                self.dock_status = 4
                time.sleep(3.0)
                self.dock_status = 0
                return
            with self.lock:
                self.s.update(docked=False, x=0.0, y=-1.0)
            self.dock_status = 3
        else:
            if self.s['docked']:
                self.dock_status = 0
                return
            self.dock_status = 2
            time.sleep(3.0)
            with self.lock:
                self.s.update(docked=True, x=-3.0, y=0.0)
            self.dock_status = 0
            self.motors = False
            self.store()

    def on_dock_stop(self, msg):
        if msg.data and self.dock_status in (1, 2):
            self.dock_status = 0 if self.s['docked'] else 3

    # --- MBF ----------------------------------------------------------------
    def do_get_path(self, goal):
        r = GetPathResult()
        r.outcome = 0
        r.path.header.frame_id = 'transit'
        tx, ty = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
        with self.lock:
            sx, sy = self.s['x'], self.s['y']
        n = max(2, int(math.hypot(tx - sx, ty - sy) / 0.2))
        for i in range(n + 1):
            r.path.poses.append(pose(sx + (tx - sx) * i / n, sy + (ty - sy) * i / n))
        r.cost = int(math.hypot(tx - sx, ty - sy) * 1000)
        self.a_get.set_succeeded(r)

    def do_exe_path(self, goal):
        poses = goal.path.poses
        rec('exe', transit=goal.path.header.frame_id == 'transit',
            first=[round(poses[0].pose.position.x, 2), round(poses[0].pose.position.y, 2)],
            last=[round(poses[-1].pose.position.x, 2), round(poses[-1].pose.position.y, 2)],
            blade=self.mower['motor'], n=len(poses))
        rate = rospy.Rate(20)
        step = SPEED * 0.05
        end = poses[-1].pose.position
        i = 0
        while i < len(poses) and not rospy.is_shutdown():
            if self.a_exe.is_preempt_requested():
                self.a_exe.set_preempted(ExePathResult(outcome=8))
                return
            # walk the poses (0.03 m apart) by one tick's distance
            left = 0 if self.holds else step
            with self.lock:
                while i < len(poses) and left > 0:
                    t = poses[i].pose.position
                    d = math.hypot(t.x - self.s['x'], t.y - self.s['y'])
                    if d <= left:
                        self.s['x'], self.s['y'] = t.x, t.y
                        left -= d
                        i += 1
                    else:
                        self.s['x'] += (t.x - self.s['x']) / d * left
                        self.s['y'] += (t.y - self.s['y']) / d * left
                        left = 0
            fb = ExePathFeedback()
            fb.dist_to_goal = math.hypot(end.x - self.s['x'], end.y - self.s['y'])
            self.a_exe.publish_feedback(fb)
            rate.sleep()
        self.a_exe.set_succeeded(ExePathResult(outcome=0))

    # --- periodic -----------------------------------------------------------
    def tick(self):
        s = self.s
        now = rospy.Time.now()
        self.tf.sendTransform((s['x'], s['y'], 0), (0, 0, 0, 1), now, 'base_link', 'map')
        self.p_dock.publish(Int8(self.dock_status))

    def pm_sleep_cmd(self, msg, field):
        self.pm_sleep[field] = msg.data
        if field in ('sleep_wait_standby', 'sleep_time_interval'):
            rec('pm_write', field=field, value=msg.data)
            self.s['pm_eeprom'] = {k: self.pm_sleep[k] for k in
                                   ('sleep_wait_standby', 'sleep_time_interval')}

    def slow(self):
        s = self.s
        online = s['docked']
        if online:
            s['battery'] = min(100.0, s['battery'] + s['charge_rate'] * 0.5)
        else:
            s['battery'] = max(0.0, s['battery'] - s['drain_rate'] * 0.5)
        pm = Power_status()
        pm.battery_capacity = int(s['battery'])
        pm.supply_status = 'ONLINE' if online else 'OFFLINE'
        pm.charger_status = 'CHARGING' if online else 'DISCHARGING'
        pm.bat_out_switch = self.bat_out
        for k, v in self.pm_sleep.items():
            setattr(pm, k, v)
        self.p_pm.publish(pm)
        self.p_in_dock.publish(Bool(s['docked']))
        o = Navi_transform()
        o.status = 'SAT'
        o.info = '0.1s,0.1s,e0.00001'
        self.p_odom.publish(o)
        self.p_motor_state.publish(Bool(self.motors))
        self.p_points.publish(StringList(string_list=['DOCK', 'UNDOCK']))
        m = Mower()
        m.max_height = 60
        m.current_height = self.mower['height']
        m.status = 'READY' if self.mower['power'] else 'OFF'
        spins = self.mower['motor'] and self.mower['power'] and not s['blade_dead']
        if spins and self.mower['rpm'] > 0:
            m.status = 'RUN'
        if s.get('mower_temp') and self.mower['power']:
            m.status = 'TEMP'
        m.moto_rpm = self.mower['rpm'] if spins else 0
        m.setpoint_rpm = self.mower['rpm']
        if not s['mower_silent']:
            self.p_mower.publish(m)
        self.silent_t = (self.silent_t or time.time()) if s['mower_silent'] else None
        if s.get('link_node'):      # node_mower_link
            self.p_link.publish(String(json.dumps(
                {'state': 'silent' if self.silent_t else 'ok', 'status': m.status,
                 'silent_s': round(time.time() - self.silent_t, 1) if self.silent_t else 0.2})))
        if not s['no_weather']:
            r = RainAlert()
            r.time = rospy.Time.now() - rospy.Duration(7200 if s['stale_weather'] else 0)
            r.rain_now = int(s['rain_now'])
            r.status_now = 'RAIN' if s['rain_now'] else 'OK'
            for f in ('60', '50', '40', '30', '20', '10'):
                setattr(r, 'status_past%sm' % f, 'RAIN' if s['past_rain'] else 'OK')
            for f in ('10', '20', '30'):
                setattr(r, 'status_nowcast%sm' % f, 'RAIN' if s['forecast_rain'] else 'OK')
            self.p_rain.publish(r)


if __name__ == '__main__':
    rospy.init_node('fake_world')
    w = World()
    rec('world_up')
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
