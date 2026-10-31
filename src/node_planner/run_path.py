"""What /web_plan/program_marker draws: the running program, the manual Show.

The marker topic is latched and every message is the whole picture (DELETEALL
first), so a client that connects during a run gets the same map as one that
watched it start. No rospy here: node_planner feeds the state and publishes.
"""
import json
import math
import re

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# /mower_smach/status of a mission that waits for Reset (top_level.py): the latched
# active_program keeps the name there and busy may lag by up to 0.5 s.
ENDED_STATUS = ('Stopped', 'TERMINAL ERROR')
IDLE_STATUS = ('', 'Ready', 'Starting...')
FRESH_STATUS = 'Initiating'          # a new Run: last_result is the previous run
STARTING_STATUS = (FRESH_STATUS, 'Initiating unfinished')

# Above this the paths are thinned: every message goes as JSON over rosbridge.
MAX_SEGMENTS = 30000
CHECKPOINT = re.compile(r'^failed: on_path-(.+)$')

ZONE_COLORS = ((0.05, 0.79, 0.94),   # #0dcaf0, the UI accent
               (1.00, 0.76, 0.03),
               (0.91, 0.40, 1.00),
               (0.55, 0.94, 0.35))
DONE_COLOR = (0.58, 0.62, 0.66)
# layer: width in m (scale.x), min. width in px (scale.y, map_view.js), brightness.
# Opaque on purpose: a blended ribbon shows every joint of a dense path.
LAYERS = {'border': (0.04, 1.5, 0.75),
          'done': (0.05, 1.5, 1.0),
          'fill': (0.05, 2.0, 0.8),
          'outline': (0.08, 3.0, 1.0)}
LAYER_ORDER = ('border', 'done', 'fill', 'outline')


def running_program(busy, program, status):
    """Name of the program the mission runs now, '' when none.

    The reading of mower_phase in vitulus_agent execute.py (a robot function must
    not depend on the agent). busy comes up to 2 s after the Run, so without it
    the status decides. A program that ended has a blank name; one sent home by
    battery / weather keeps it and counts until the dock (status Ready).
    """
    program = (program or '').strip()
    status = (status or '').strip()
    if not program or status in ENDED_STATUS:
        return ''
    if busy or status not in IDLE_STATUS:
        return program
    return ''


def parse_checkpoint(last_result):
    """(zone, path index, pose index) of 'failed: on_path-<zone>-<path>-<window>',
    read as the mission reads it on resume, or None."""
    found = CHECKPOINT.match((last_result or '').strip())
    if not found:
        return None
    parts = found.group(1).rsplit('-', 2)
    try:
        if len(parts) == 3:
            return parts[0], int(parts[1]), int(parts[2])
        if len(parts) == 2:
            return parts[0], int(parts[1]), 0
    except ValueError:
        pass
    return None


def _xy(pose):
    return round(pose.pose.position.x, 2), round(pose.pose.position.y, 2)


def _segments(xy):
    return list(zip(xy[:-1], xy[1:]))


def _length(segments):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in segments)


def _flat(segments):
    points = []
    for (ax, ay), (bx, by) in segments:
        points.append(Point(ax, ay, 0.0))
        points.append(Point(bx, by, 0.0))
    return points


def _line_marker(ns, marker_id, layer, color, segments, stamp):
    width, min_px, level = LAYERS[layer]
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.scale.x = width
    marker.scale.y = min_px
    marker.scale.z = width
    marker.color = ColorRGBA(color[0] * level, color[1] * level, color[2] * level, 1.0)
    marker.pose.orientation.w = 1.0
    marker.points = _flat(segments)
    return marker


def _thin(xy, step):
    if step <= 1 or len(xy) <= 2:
        return xy
    return xy[::step] + ([xy[-1]] if (len(xy) - 1) % step else [])


def _done_poses(zone_i, path_i, zones, checkpoint):
    """How many leading poses of a path the mission already went through."""
    if checkpoint is None:
        return 0
    name, at_path, at_pose = checkpoint
    at_zone = next((i for i, z in enumerate(zones) if z.name == name), None)
    if at_zone is None or not 0 <= at_path < len(zones[at_zone].paths):
        return 0                    # an older path set, as the mission treats it
    if (zone_i, path_i) < (at_zone, at_path):
        return len(zones[zone_i].paths[path_i].poses)
    if (zone_i, path_i) == (at_zone, at_path):
        return at_pose + 1
    return 0


def build_markers(program, stamp, checkpoint=None, prefix='run'):
    """Markers of a program, at most one LINE_LIST per zone and layer: border,
    done, fill (open coverage passes), outline (closed rings along the border).
    Returns (markers, done metres, total metres). The path poses are what the
    mission follows; zones and paths go in the order it mows them. `prefix`
    keeps the markers of two programs on one map apart (ns + id is the key)."""
    zones = list(program.zone_list)
    count = sum(max(len(p.poses) - 1, 0) for z in zones for p in z.paths)
    step = int(math.ceil(count / float(MAX_SEGMENTS))) if count > MAX_SEGMENTS else 1
    markers = []
    done_m = left_m = 0.0
    for zone_i, zone in enumerate(zones):
        color = ZONE_COLORS[zone_i % len(ZONE_COLORS)]
        layers = {name: [] for name in LAYER_ORDER}
        ring = [(round(p.x, 2), round(p.y, 2)) for p in zone.polygon.polygon.points]
        if len(ring) > 1:
            layers['border'] = _segments(ring + ring[:1])
        for path_i, path in enumerate(zone.paths):
            xy = [_xy(pose) for pose in path.poses]
            if len(xy) < 2:
                continue
            closed = xy[0] == xy[-1]
            done_n = min(_done_poses(zone_i, path_i, zones, checkpoint), len(xy))
            done = _thin(xy[:done_n], step) if done_n > 1 else []
            left = _thin(xy[max(done_n - 1, 0):], step)
            if done:
                layers['done'].extend(_segments(done))
                done_m += _length(_segments(done))
            if len(left) > 1:
                layers['outline' if closed else 'fill'].extend(_segments(left))
                left_m += _length(_segments(left))
        for name in LAYER_ORDER:
            if layers[name]:
                markers.append(_line_marker(
                    '{}_{}_{}'.format(prefix, zone_i, name), LAYER_ORDER.index(name), name,
                    DONE_COLOR if name == 'done' else color, layers[name], stamp))
    return markers, done_m, done_m + left_m


class ShownPaths(object):
    """What is on the map. A run has priority over a manual Show:

    - a run starts: its paths appear (a Hide of the previous run is forgotten);
      a manual Show of the same program is taken over by the run and ends with
      it, a manual Show of another program waits and comes back after the run;
    - Hide during a run holds until Show or the end of that run;
    - a Show during a run goes on top of the run, the run stays.
    """

    def __init__(self):
        self.run = ''
        self.run_hidden = False
        self.manual = None           # name of the manual Show
        self.manual_markers = []
        self.manual_waits = False

    def set_run(self, name):
        """The mission's program. True when it changed (a run began or ended)."""
        if name == self.run:
            return False
        self.run = name
        self.run_hidden = False
        if not name:
            self.manual_waits = False
        elif self.manual == name:
            self.hide_manual()
        else:
            self.manual_waits = self.manual is not None
        return True

    def show_manual(self, name, markers):
        if name == self.run:
            self.run_hidden = False
            return
        self.manual, self.manual_markers, self.manual_waits = name, markers, False

    def hide_manual(self):
        self.manual, self.manual_markers, self.manual_waits = None, [], False

    def set_run_visible(self, visible):
        if self.run:
            self.run_hidden = not visible

    def manual_visible(self):
        return [] if self.manual is None or self.manual_waits else self.manual_markers

    def state_json(self, done_m=0.0, total_m=0.0):
        """The latched /web_plan/program_marker_state: the Show button and the
        running card read it, so every client shows the real state."""
        return json.dumps({
            'run': self.run,
            'run_visible': bool(self.run) and not self.run_hidden,
            'manual': '' if self.manual is None or self.manual_waits else self.manual,
            'done_m': round(done_m, 1),
            'total_m': round(total_m, 1)}, sort_keys=True)
