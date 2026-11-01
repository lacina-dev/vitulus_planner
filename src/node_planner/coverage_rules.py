#!/usr/bin/env python3
"""coverage_rules.py -- pure helpers behind map rules in coverage, the zone dry
run and zone split/merge (docs: vitulus_mapping docs/MAP_OBJECTS.md, Coverage and
section 6).

NO rospy and no ROS messages: numpy, cv2 and shapely only, so all of it can be
tested without a master. Coordinates are map-frame metres unless a name says px.

What a rule does to coverage: `nogo` and `noblade` areas become OBSTACLE in
np_zone only. np_zone_navi stays as it is, so a noblade area remains passable.
The stamp is grown by a margin, because a path runs through the robot centre: at
the obstacle margin the footprint just clears the area, so the blade guard stays
quiet along it. nogo gets NOGO_EXTRA_M on top, because the rules guard tapers the
speed to zero 0.15 m before the footprint touches a nogo.
"""
import hashlib
import json
import math

import cv2
import numpy as np
from shapely import geometry, ops

CUT_HEIGHT_MIN = 20     # mm, as the mower settings
CUT_HEIGHT_MAX = 80     # mm, deck maximum

COVERAGE_TYPES = ('nogo', 'noblade')
NOGO_EXTRA_M = 0.15
ZONE_PAD_M = 1.0        # a rule this close to a zone's box can reach its paths

# max_vel_x of navi_man per program speed, and the share of it a mowing run keeps
# over its length (turns, starts). Fitted on real runs: slow 0.43, mid 0.54.
SPEED_MPS = {'slow': 0.25, 'mid': 0.40, 'fast': 0.75}
MOWING_EFFICIENCY = 0.45

PREVIEW_MAX_POINTS = 3000
MERGE_GAP_M = 0.05      # zones closer than twice this count as adjacent
MIN_HOLE_M2 = 0.05      # a smaller hole in a merge is a drawing artefact
MIN_PART_M2 = 0.05      # smaller = a sliver of a split, not a part


# (en, cs): what a start tells about one zone of the program, chain style.
START_NOTES = {
    'zone_under_rule': (
        "zone '{zone}' is left out: map rule '{rule}' leaves it no mowing path",
        "zóna {zone} se vynechá: pravidlo mapy {rule} v ní nenechá žádnou trasu"),
    'checkpoint_stale': (
        "zone '{zone}' has other paths than the stopped run, mowing it again from "
        "its start",
        "zóna {zone} má jiné trasy než přerušený běh, poseká se znovu od začátku"),
    'checkpoint_zone_gone': (
        "zone '{zone}' of the stopped run does not run now, mowing the whole program",
        "zóna {zone} přerušeného běhu teď nejede, poseká se celý program"),
}


def start_note(code, program, zone, **params):
    en, cs = START_NOTES[code]
    return {'program': program, 'zone': zone, 'code': code,
            'text_en': en.format(zone=zone, **params),
            'text_cs': cs.format(zone=zone, **params)}


def path_fingerprint(paths):
    """Identity of a zone's paths (nav_msgs/Path list), to 1 mm. A checkpoint is
    a path index: it means something only on the paths it was taken on."""
    h = hashlib.md5()
    for path in paths:
        h.update(b'|')
        for q in path.poses:
            h.update('{:.3f},{:.3f};'.format(q.pose.position.x,
                                             q.pose.position.y).encode())
    return h.hexdigest()


class Cancelled(Exception):
    """A dry run was superseded by a newer request."""


def problem(code, severity='error', **params):
    """Machine form of a finding: the UI translates `code` and uses `params`."""
    return {'code': code, 'severity': severity, 'params': params}


def has_errors(problems):
    return any(p['severity'] == 'error' for p in problems)


# ---- cut height -------------------------------------------------------------
def cut_height_ok(value):
    """0 = not set (the mission falls back to the zone, then the mower)."""
    return value == 0 or CUT_HEIGHT_MIN <= value <= CUT_HEIGHT_MAX


def effective_cut_height(program, zone_height):
    """The height the mission really uses for a zone (GetZoneData)."""
    own = program.cut_height if getattr(program, 'override_zone', False) else 0
    return own if own > 0 else zone_height


def cut_height_text():
    return '{}-{} mm'.format(CUT_HEIGHT_MIN, CUT_HEIGHT_MAX)


# ---- rules ------------------------------------------------------------------
def parse_rules(text):
    """/map_objects/rules payload -> (site, rules that shape coverage). Rules with
    fewer than 3 points cannot enclose anything and are left out. ValueError on a
    payload that is not the contract's object."""
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError('expected a JSON object')
    rules = []
    for r in data.get('rules') or []:
        if not isinstance(r, dict) or r.get('type') not in COVERAGE_TYPES:
            continue
        points = [(float(p[0]), float(p[1])) for p in r.get('points') or []]
        if len(points) >= 3:
            rules.append({'type': r['type'], 'name': r.get('name') or '',
                          'points': points})
    return data.get('site'), rules


def rules_of_zone(rules, points, pad=ZONE_PAD_M):
    """The rules that can touch a zone, by bounding box."""
    if not rules or not points:
        return []
    x0 = min(p[0] for p in points) - pad
    x1 = max(p[0] for p in points) + pad
    y0 = min(p[1] for p in points) - pad
    y1 = max(p[1] for p in points) + pad
    return [r for r in rules
            if min(p[0] for p in r['points']) <= x1
            and max(p[0] for p in r['points']) >= x0
            and min(p[1] for p in r['points']) <= y1
            and max(p[1] for p in r['points']) >= y0]


def fingerprint(rules):
    """Identity of what the rules do to coverage: type and shape to 1 cm, in any
    order. seq, stamp, ids and names do not move a path. '' = no rules, which is
    also what a zone from before this feature counts as."""
    if not rules:
        return ''
    canon = sorted((r['type'], [(round(x, 2), round(y, 2)) for x, y in r['points']])
                   for r in rules)
    return hashlib.md5(json.dumps(canon).encode()).hexdigest()


def stamp(np_zone, rules, to_px, margin_px, extra_nogo_px, value):
    """Write `rules` into np_zone as `value`, in place, grown by margin_px (plus
    extra_nogo_px for nogo). to_px(x, y) -> (col, row)."""
    for kind, grow in (('noblade', margin_px), ('nogo', margin_px + extra_nogo_px)):
        polys = [np.array([to_px(x, y) for x, y in r['points']], dtype=np.int32)
                 for r in rules if r['type'] == kind]
        if not polys:
            continue
        mask = np.zeros(np_zone.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, pts=polys, color=255)
        if grow > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                               (2 * grow + 1, 2 * grow + 1))
            mask = cv2.dilate(mask, kernel, iterations=1)
        np_zone[mask > 0] = value
    return np_zone


# ---- shapes -----------------------------------------------------------------
def _points(raw):
    """[[x, y], ...] from JSON -> [(x, y), ...]. ValueError on anything else."""
    try:
        points = [(float(p[0]), float(p[1])) for p in raw]
    except (TypeError, IndexError):
        raise ValueError('bad points')
    if not all(math.isfinite(c) for p in points for c in p):
        raise ValueError('bad points')
    return points


def read_shape(raw):
    """(points, polygon, problems) of a requested zone outline. polygon is None
    when the outline is unusable."""
    try:
        points = _points(raw)
    except ValueError:
        return [], None, [problem('bad_points')]
    if len(points) < 3:
        return points, None, [problem('bad_points')]
    poly = geometry.Polygon(points)
    if not poly.is_valid:
        return points, None, [problem('self_intersecting')]
    if poly.area <= 0:
        return points, None, [problem('bad_points')]
    return points, poly, []


def settings_problems(props):
    """Zone settings that cannot give a usable result."""
    out = []
    if not cut_height_ok(props['cut_height']):
        out.append(problem('cut_height_range', value=props['cut_height'],
                           min=CUT_HEIGHT_MIN, max=CUT_HEIGHT_MAX))
    if props['border_paths'] < 1:
        out.append(problem('border_paths_range', value=props['border_paths'], min=1))
    # the planner runs round(0.6 / distance) passes: 0 passes above 1.2 m
    if not 0 < props['paths_distance'] <= 1.2:
        out.append(problem('paths_distance_range', value=props['paths_distance'],
                           max=1.2))
    if props['rpm'] < 0:
        out.append(problem('rpm_range', value=props['rpm'], min=0))
    return out


def valid_parts(poly):
    """A polygon as valid polygons: itself when it already is one."""
    if poly.is_valid:
        return [poly]
    fixed = poly.buffer(0)
    return [g for g in getattr(fixed, 'geoms', [fixed])
            if g.geom_type == 'Polygon' and not g.is_empty]


def overlap_area(shape, points):
    """m2 of shape inside the outline `points`."""
    other = geometry.Polygon(points).buffer(0)
    return shape.intersection(other).area if shape.intersects(other) else 0.0


def outline(poly):
    """Polygon -> [(x, y), ...] without the closing vertex."""
    return [(float(x), float(y)) for x, y in poly.exterior.coords[:-1]]


def _split_parts(poly, line):
    return [g for g in ops.split(poly, line).geoms
            if g.geom_type == 'Polygon' and g.area >= MIN_PART_M2]


def _extended(points, poly):
    """The line with both ends run on past the zone."""
    x0, y0, x1, y1 = poly.bounds
    reach = math.hypot(x1 - x0, y1 - y0) + 1.0

    def beyond(a, b):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        return (b[0] + (b[0] - a[0]) / d * reach, b[1] + (b[1] - a[1]) / d * reach)
    return [beyond(points[1], points[0])] + points + [beyond(points[-2], points[-1])]


def split_zone(zone_points, raw_line):
    """Split an outline by a line that need not reach across it. Returns (parts,
    None) with exactly two outlines, larger first, or (None, code)."""
    try:
        line = _points(raw_line)
    except ValueError:
        return None, 'bad_line'
    if len(line) < 2 or len(set(line)) < 2 or line[0] == line[1] or line[-2] == line[-1]:
        return None, 'bad_line'
    poly = geometry.Polygon(zone_points)
    if not poly.is_valid:
        return None, 'self_intersecting'
    parts = _split_parts(poly, geometry.LineString(line))
    if len(parts) != 2:
        parts = _split_parts(poly, geometry.LineString(_extended(line, poly)))
    if len(parts) < 2:
        return None, 'line_misses_zone'
    if len(parts) != 2:
        return None, 'split_not_two_parts'
    parts.sort(key=lambda g: g.area, reverse=True)
    return [outline(g) for g in parts], None


def merge_zones(outlines):
    """One outline of zones that touch or overlap, or that lie at most
    2 * MERGE_GAP_M apart. Returns (points, None) or (None, code)."""
    polys = [geometry.Polygon(p) for p in outlines]
    if not all(p.is_valid for p in polys):
        return None, 'self_intersecting'
    grown = [p.buffer(MERGE_GAP_M, join_style='mitre') for p in polys]
    merged = ops.unary_union(grown).buffer(-MERGE_GAP_M, join_style='mitre')
    if merged.geom_type != 'Polygon':
        return None, 'not_adjacent'
    if any(geometry.Polygon(r).area >= MIN_HOLE_M2 for r in merged.interiors):
        return None, 'merge_has_hole'     # a zone has no holes: it would mow them
    return outline(geometry.Polygon(merged.exterior).simplify(0.01)), None


def unique_name(base, taken):
    """base, or the first of 'base 2', 'base 3'... that is free (case-blind)."""
    folded = {n.casefold() for n in taken}
    if base.casefold() not in folded:
        return base
    n = 2
    while '{} {}'.format(base, n).casefold() in folded:
        n += 1
    return '{} {}'.format(base, n)


# ---- dry-run figures --------------------------------------------------------
def estimate_minutes(length_m, speed, runs, zone=None):
    """(minutes, source). runs: [{length, minutes, speed, zones, result}] of past
    programs. Finished runs at this speed give the rate, those that mowed this
    zone first; otherwise (source 'speed') the speed table."""
    speed = (speed or 'mid').lower()
    if speed not in SPEED_MPS:
        speed = 'mid'
    usable = [r for r in runs
              if r['length'] > 0 and r['minutes'] > 0 and r['speed'] == speed
              and r['result'] == 'succeeded']
    own = [r for r in usable if zone and zone in r['zones']]
    picked = own or usable
    if picked:
        rate = (float(sum(r['minutes'] for r in picked))
                / sum(r['length'] for r in picked))
        return max(1, int(round(rate * length_m))), 'history'
    mps = SPEED_MPS[speed] * MOWING_EFFICIENCY
    return max(1, int(round(length_m / mps / 60.0))), 'speed'


def thin_preview(polylines, max_points=PREVIEW_MAX_POINTS):
    """Polylines rounded to cm, without repeated points, at most about max_points
    in all. Every line keeps its two ends."""
    clean = []
    for line in polylines:
        out = []
        for x, y in line:
            p = [round(float(x), 2), round(float(y), 2)]
            if not out or out[-1] != p:
                out.append(p)
        if out:
            clean.append(out)
    total = sum(len(line) for line in clean)
    if total <= max_points:
        return clean
    keep = float(max_points) / total
    thinned = []
    for line in clean:
        n = max(2, int(len(line) * keep))
        if n >= len(line):
            thinned.append(line)
            continue
        step = (len(line) - 1.0) / (n - 1)
        thinned.append([line[int(round(i * step))] for i in range(n)])
    return thinned


def _label_near(labels, col, row, reach_px):
    """Label of the free cell nearest to (col, row) within reach_px, or 0."""
    h, w = labels.shape
    r0, r1 = max(0, row - reach_px), min(h, row + reach_px + 1)
    c0, c1 = max(0, col - reach_px), min(w, col + reach_px + 1)
    if r0 >= r1 or c0 >= c1:
        return 0
    rows, cols = np.nonzero(labels[r0:r1, c0:c1])
    if rows.size == 0:
        return 0
    i = np.argmin((rows + r0 - row) ** 2 + (cols + c0 - col) ** 2)
    return int(labels[rows[i] + r0, cols[i] + c0])


def cut_off_parts(parts, free, to_px, home=None, reach_px=40):
    """Indices of the zone parts the robot cannot drive to. free: bool raster of
    navigable cells. home: (x, y) it starts from; without one (or when it lies
    off every free cell) the largest part stands in. A part whose cells cannot be
    found counts as reachable: this reports, it never hides."""
    if len(parts) < 1:
        return []
    _, labels = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
    of_part = []
    for part in parts:
        coords = list(part.exterior.coords)[::5]
        rep = part.representative_point()
        coords.append((rep.x, rep.y))
        seen = {}
        for x, y in coords:
            col, row = to_px(x, y)
            label = _label_near(labels, col, row, 2)
            if label:
                seen[label] = seen.get(label, 0) + 1
        of_part.append(max(seen, key=seen.get) if seen else 0)
    target = 0
    if home is not None:
        col, row = to_px(home[0], home[1])
        target = _label_near(labels, col, row, reach_px)
    if not target:
        target = of_part[max(range(len(parts)), key=lambda i: parts[i].area)]
    return [i for i, label in enumerate(of_part) if label and label != target]
