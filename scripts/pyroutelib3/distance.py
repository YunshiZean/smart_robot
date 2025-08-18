# © Copyright 2024 Mikołaj Kuranowski
# SPDX-License-Identifier: GPL-3.0-or-later

import math

from .protocols import DistanceFunction, Position

EARTH_RADIUS = 6371.0088
"""Mean radius of Earth, in kilometers.
Source: https://en.wikipedia.org/wiki/Earth_radius#Arithmetic_mean_radius
"""

EARTH_DIAMETER = EARTH_RADIUS + EARTH_RADIUS
"""Mean diameter of Earth, in kilometers.
Source: https://en.wikipedia.org/wiki/Earth_radius#Arithmetic_mean_radius
"""


euclidean_distance: DistanceFunction = math.dist
"""Calculates the `Euclidean distance <https://en.wikipedia.org/wiki/Euclidean_distance>`_
between two points, in the same units as the input positions.
"""


def taxicab_distance(a: Position, b: Position) -> float:
    """Calculates the `Taxicab distance <https://en.wikipedia.org/wiki/Taxicab_geometry>`_
    between two points, in the same units as the input positions."""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def haversine_earth_distance(a: Position, b: Position) -> float:
    """Calculates the great-circle distance between two lat-lon positions
    on Earth using the `haversine formula <https://en.wikipedia.org/wiki/Haversine_formula>`_.
    Returns the result in kilometers.
    """

    lat1: float = math.radians(a[0])
    lon1: float = math.radians(a[1])
    lat2: float = math.radians(b[0])
    lon2: float = math.radians(b[1])

    sin_dlat_half = math.sin((lat2 - lat1) * 0.5)
    sin_dlon_half = math.sin((lon2 - lon1) * 0.5)

    h = (
        sin_dlat_half * sin_dlat_half
        + math.cos(lat1) * math.cos(lat2) * sin_dlon_half * sin_dlon_half
    )

    return EARTH_DIAMETER * math.asin(math.sqrt(h))


import numpy as np


# 点到直线的最小距离
def point_distance_line(p, p1, p2):
    p = np.array(p)
    p1 = np.array(p1)
    p2 = np.array(p2)

    if np.equal(p1, p2).all():
        return np.linalg.norm(p - p1)

    return np.linalg.norm(np.cross(p2 - p1, p1 - p)) / np.linalg.norm(p2 - p1)

# 点到线段的最小距离
# https://blog.csdn.net/ao1886/article/details/116461824
def dis_point_to_seg_line(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)  # trans to np.array
    d = np.divide(b - a, np.linalg.norm(b - a))  # normalized tangent vector
    s = np.dot(a - p, d)  # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0])  # clamped parallel distance
    c = np.cross(p - a, d)  # perpendicular distance component
    return np.hypot(h, np.linalg.norm(c))
