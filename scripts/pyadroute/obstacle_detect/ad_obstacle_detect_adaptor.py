import shapely
from shapely import (
    LineString,
    Polygon,
    get_coordinates,
    offset_curve,
    total_bounds,
    unary_union,
)
import numpy as np
import math

from ..utils.logger import get_logger

from ..path_wraper import PathWrapper
from ..state import State

from ..utils.point import point_in_bounds, get_rectangle
from ..utils.path import (
    distance_ahead_path,
    nearest_point_of_point_to_path,
    next_point_extend_distance,
    euclidean_distance,
)
from sensor_msgs.msg import PointCloud2

# from shapely.ops import nearest_points
from shapely.geometry import Point, LinearRing
from sensor_msgs import point_cloud2


logger = get_logger("AdObstacleDetectAdaptor")


class AdObstacleDetectAdaptor:

    def __init__(self, offset):
        self.offset = offset
        self.max_distance = 0.0
        self.path = None
        self.point_cloud_data = None
        self.point_start = self.point_target_idx = None
        self.detect_area = None

    def set_pathw(self, pathw: PathWrapper, offset: float):
        self.offset = offset

        if pathw is None or pathw.length() == 0:
            self.path = None
        else:
            self.path = pathw.path
            # line_path = LineString(pathw.path)
            # self.left_line = get_coordinates(offset_curve(line_path, offset)).tolist()
            # self.right_line = get_coordinates(offset_curve(line_path, -offset)).tolist()
        self.detect_area = None

    def set_pointcloud(self, point_cloud: PointCloud2):
        # self.point_cloud = point_cloud
        self.point_cloud_data = point_cloud2.read_points(
            point_cloud, skip_nans=True, field_names=("x", "y")
        )

        # self.point_cloud_data = [shapely.Point(p[0], p[1]) for p in point_cloud_data]

    def get_detect_area_coords(self):
        detect_area = self.detect_area
        if detect_area is not None:
            return get_coordinates(detect_area).tolist()
        return None

    # detec Area:
    #      p1--p2 p5-------------------------------------p6
    #  car |---|nearest_point_index--------|target_idx----|max_distance
    #      p4--p3   -------------------------------------
    def detect(self, state: State, nearest_point_index, point_target_idx, max_distance):
        if state is None or self.path is None:
            return False

        if point_target_idx is None:
            return False

        point_start = (state.x, state.y)
        detected = False
        if (
            self.point_start != point_start
            or self.point_target_idx != point_target_idx
            or self.max_distance != max_distance
            or self.detect_area is None
        ):
            self.point_start = point_start
            self.point_target_idx = point_target_idx
            self.max_distance = max_distance

            detect_area1 = None
            detect_area2 = None

            linepoint = self.path[nearest_point_index]

            # calc p1 p2 p3 p4
            p1, p2, p3, p4 = get_rectangle(point_start, linepoint, self.offset)
            detect_area1 = Polygon([p1, p2, p3, p4])

            distance_sum = euclidean_distance(point_start, linepoint)

            if distance_sum < self.max_distance:
                # calc p5 p6 p7 p8
                p5_idx = nearest_point_index
                p6_idx = point_target_idx
                distance = distance_ahead_path(
                    self.path, nearest_point_index, point_target_idx
                )
                distance_sum += distance
                remain_distance = self.max_distance - distance_sum
                if remain_distance > 0:
                    _, idx = next_point_extend_distance(
                        point_target_idx, remain_distance, self.path
                    )
                    p6_idx = idx

                if p6_idx > p5_idx:

                    line_path = LineString(self.path[p5_idx : p6_idx + 1])
                    left_line = get_coordinates(
                        offset_curve(line_path, self.offset)
                    ).tolist()
                    right_line = get_coordinates(
                        offset_curve(line_path, -self.offset)
                    ).tolist()
                    detect_area_lines = []
                    detect_area_lines.extend(left_line)
                    detect_area_lines.extend(reversed(right_line))
                    if len(detect_area_lines) >= 4:
                        detect_area2 = Polygon(detect_area_lines)

            if detect_area1 is not None and detect_area2 is not None:
                try:
                    self.detect_area = unary_union([detect_area1, detect_area2])
                except:
                    self.detect_area = detect_area2

            elif detect_area1 is not None:
                self.detect_area = detect_area1
            elif detect_area2 is not None:
                self.detect_area = detect_area2
            else:
                self.detect_area = None

        # logger.info("detect_area %s:", self.detect_area)

        if self.detect_area is not None:
            bounds = total_bounds(self.detect_area).tolist()
            point_cloud_data = self.point_cloud_data  # for thread safe
            if point_cloud_data is not None:
                for p in point_cloud_data:
                    # contains cover within intersects
                    if point_in_bounds(p, bounds) and self.detect_area.intersects(
                        shapely.Point(p)
                    ):
                        print("obstacle " + str(p))
                        detected = True
                        break

        return detected
