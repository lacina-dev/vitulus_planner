#!/usr/bin/env python3
"""
bundle_sync.py -- pure conversion helpers for node_planner <-> site bundle.

NO rospy / ROS-message imports.  Only ``math`` + the pure-python
vitulus_mapping libs (bundle, geo).  This keeps the geometry conversion
unit-testable offline (no ROS master) and lets node_planner import it behind a
try/except so a missing/unbuilt vitulus_mapping never breaks the legacy path.

Zone geometry lives in the robot MAP frame inside node_planner (metres); the
site bundle stores it canonically in UTM (portable across maps/datums).  These
helpers do the map<->UTM polygon conversion through a datum and marshal
zone/program metadata to/from the plain dict shapes that vitulus_mapping.bundle
(the FROZEN API) reads and writes.

Datum authority (node_planner):  the ACTIVE navi map's datum -- the same datum
that drives the live utm<->map TF (navi_man set_map_utm_transform), taken from
the map's maps_data pickle.  Using one datum for BOTH export and import makes
map->UTM->map an exact round-trip regardless of any site datum.yaml drift.
"""

import math

from vitulus_mapping import bundle, geo

__all__ = [
    "ZONE_PROP_FIELDS", "PROGRAM_SCALAR_FIELDS",
    "resolve_site",
    "polygon_map_to_utm", "polygon_utm_to_map",
    "zone_to_bundle", "bundle_to_polygon_map",
    "program_to_bundle", "rel_mismatch",
]

# MapEditZone scalar fields carried into zones.geojson props ('name' is stored
# by bundle.save_zones separately).  area/length are the coverage metrics used
# by the import-time consistency guard.
ZONE_PROP_FIELDS = ("area", "length", "cut_height", "rpm", "type",
                    "border_paths", "coverage_angle", "paths_distance",
                    "simplify")

# PlannerProgram scalar fields carried into programs.yaml.  zone_list is stored
# as zone_names[] (geometry is regenerated from zones.geojson on import).
# map_name is intentionally omitted -- the site implies it.
PROGRAM_SCALAR_FIELDS = ("speed", "rpm", "cut_height", "override_zone",
                         "last_result", "area", "length",
                         "last_duration_minutes")


# --------------------------------------------------------------------------
# site resolution  (manifest.navi_map == active navi map name)
# --------------------------------------------------------------------------
def resolve_site(active_map, override=None):
    """Return the bundle site whose manifest 'navi_map' == active_map, or the
    override site (if it exists), else None.  Pure: scans bundle.SITES_ROOT."""
    if override:
        return override if bundle.site_exists(override) else None
    if not active_map:
        return None
    for site in bundle.list_sites():
        try:
            m = bundle.load_manifest(site)
        except Exception:
            m = None
        if m and m.get("navi_map") == active_map:
            return site
    return None


# --------------------------------------------------------------------------
# polygon conversion  (map frame metres <-> UTM metres)
# --------------------------------------------------------------------------
def polygon_map_to_utm(points_xy, datum):
    """[(x, y), ...] MAP-frame metres -> [(e, n), ...] UTM metres."""
    return [geo.map_to_utm(float(x), float(y), datum) for (x, y) in points_xy]


def polygon_utm_to_map(points_en, datum):
    """[(e, n), ...] UTM metres -> [(x, y), ...] MAP-frame metres."""
    return [geo.utm_to_map(float(e), float(n), datum) for (e, n) in points_en]


def zone_to_bundle(name, polygon_xy, props, datum):
    """Build a bundle zone dict {name, polygon(UTM), props} from a map-frame
    polygon (list of (x, y)) plus a scalar props dict."""
    return {"name": name,
            "polygon": polygon_map_to_utm(polygon_xy, datum),
            "props": dict(props)}


def bundle_to_polygon_map(bundle_zone, datum):
    """UTM polygon of a bundle zone dict -> [(x, y), ...] MAP-frame points."""
    return polygon_utm_to_map(bundle_zone.get("polygon", []), datum)


# --------------------------------------------------------------------------
# program marshalling
# --------------------------------------------------------------------------
def program_to_bundle(name, zone_names, scalars):
    """Build a programs.yaml dict.  scalars: dict of PROGRAM_SCALAR_FIELDS."""
    out = {"name": name, "zone_names": list(zone_names)}
    out.update(scalars)
    return out


# --------------------------------------------------------------------------
# consistency guard
# --------------------------------------------------------------------------
def rel_mismatch(regenerated, stored, tol=0.05):
    """Relative difference |regen - stored| / |stored| if it exceeds tol, else
    None.  Returns None when stored is missing/<=0 (nothing to compare)."""
    try:
        stored = float(stored)
        regenerated = float(regenerated)
    except (TypeError, ValueError):
        return None
    if stored <= 0:
        return None
    rel = abs(regenerated - stored) / abs(stored)
    return rel if rel > tol else None
