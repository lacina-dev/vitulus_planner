#!/usr/bin/env python

import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

from nav_msgs.msg import Odometry, Path
from vitulus_msgs.msg import MapEditPolygon, MapEditPolygonList, MapEditZone, MapEditZoneList, MapEditMap, PlannerProgramList, PlannerProgram
from visualization_msgs.msg import Marker, MarkerArray

# np.set_printoptions(threshold=sys.maxsize)


class MapSpec:
    ROBOT_RADIUS = 0.38
    COVERAGE_DIAMETER = 0.29
    OBSTACLE_RESERVE = 0.1
    FILL_HOLE = 2

    UNKNOWN = -1
    FREE = 0
    OBSTACLE = 100
    BORDER_PATH = 120
    NOTHING = -120


class PathPolygon:
    def __init__(self, polygon, path_level):
        self.polygon = polygon
        self.path_level = path_level


class Zone:
    def __init__(self, np_zone, np_zone_navi, msg=MapEditZone()):
        self.msg = msg
        self.np_zone = np.copy(np_zone)
        self.np_zone_navi = np.copy(np_zone_navi)
        self.np_zone_paths = np.copy(np_zone)
        self.path_polygons = []
        self.multilines = []
        self.zone_path_lines_marker_array = MarkerArray()
        self.path_msg_coverage = Path()
        self.path_msg_coverage.header.frame_id = "map"
        self.path_msg_outline = Path()
        self.path_msg_outline.header.frame_id = "map"


class MapData:
    def __init__(self, np_map, map_msg=OccupancyGrid()):
        self.name = "map"
        self.initial_map = map_msg
        self.polygon_msgs = []
        self.zones = []
        self.planner_program_list_msg = None
        self.resolution = self.initial_map.info.resolution
        self.origin_x = self.initial_map.info.origin.position.x
        self.origin_y = self.initial_map.info.origin.position.y
        self.position_rc = [self.initial_map.info.width / 2, self.initial_map.info.height / 2]
        self.x_origin = -1 * int(self.initial_map.info.origin.position.x / self.resolution)
        self.y_origin = -1 * int(self.initial_map.info.origin.position.y / self.resolution)
        self.width, self.height = np.shape(np_map)
        self.coverage_distance_px = int(round(((MapSpec.COVERAGE_DIAMETER / 2.0) + MapSpec.OBSTACLE_RESERVE) / self.resolution))
        self.obstacle_margin = MapSpec.ROBOT_RADIUS
        self.obstacle_margin_px = int(round(MapSpec.ROBOT_RADIUS / self.resolution))
        self.fill_hole_px = MapSpec.FILL_HOLE
        self.fill_shape = 'ellipse'
        self.fill_shape_cv = cv2.MORPH_ELLIPSE
        self.np_original = np.copy(np_map)
        self.np_unk = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_free = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_free_filled = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_occupied = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_unk_cleaned = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_obstacle_margin = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_assembled = np.full_like(np_map, fill_value=MapSpec.FREE, dtype=np.int8)
        self.np_semi_assembled = np.full_like(np_map, fill_value=MapSpec.FREE, dtype=np.int8)
        self.np_poly_obstacle = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_poly_free = np.full_like(np_map, fill_value=MapSpec.NOTHING, dtype=np.int8)

    def clear_layers(self):
        self.np_unk = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_free = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_free_filled = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_occupied = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_unk_cleaned = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_obstacle_margin = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_assembled = np.full_like(self.np_original, fill_value=MapSpec.FREE, dtype=np.int8)
        self.np_semi_assembled = np.full_like(self.np_original, fill_value=MapSpec.FREE, dtype=np.int8)
        self.np_poly_obstacle = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)
        self.np_poly_free = np.full_like(self.np_original, fill_value=MapSpec.NOTHING, dtype=np.int8)

