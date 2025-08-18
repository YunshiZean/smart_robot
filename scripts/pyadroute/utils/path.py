import numpy as np
import math

from ..path_wraper import PathWrapper

from .angle import angle_mod
from ..protocols import DistanceFunction, Position
from scipy.interpolate import splprep, splev
from scipy.interpolate import make_interp_spline

euclidean_distance: DistanceFunction = math.dist


# 点到线段的最小距离
# https://blog.csdn.net/ao1886/article/details/116461824
def distance_point_to_segment(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)  # trans to np.array
    d = np.divide(b - a, np.linalg.norm(b - a))  # normalized tangent vector
    s = np.dot(a - p, d)  # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0])  # clamped parallel distance
    c = np.cross(p - a, d)  # perpendicular distance component
    return np.hypot(h, np.linalg.norm(c))


# 计算点到线段的最近点
# 输入点P(x0,y0)和线段AB（x1,y1,x2,y2）,输出点到线段的最近点
# https://zhuanlan.zhihu.com/p/526154079
def nearest_point_of_point_to_segment(x0, y0, x1, y1, x2, y2) -> Position:
    # 如果两点相同，则输出一个点的坐标为垂足
    if x1 == x2 and y1 == y2:
        return x1, y1
    k = -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1)) / (
        (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
    )

    if k <= 0:
        return x1, y1
    elif k >= 1:
        return x2, y2
    else:
        xf = k * (x2 - x1) + x1
        yf = k * (y2 - y1) + y1
        return xf, yf


def refine_path(path, path_attr, step=0.01):
    if path is None or len(path) == 0:
        return None, None

    new_path = []
    new_path_attr = []
    new_path.append(path[0])
    new_path_attr.append(path_attr[0])

    for u, v, tag in zip(path[:-1], path[1:], path_attr[1:]):
        # print("uv:", u, v)
        num = math.ceil(max(abs(u[0] - v[0]), abs(u[1] - v[1])) / step)
        if num > 1:
            path_x = np.linspace(u[0], v[0], num, endpoint=True)
            path_y = np.linspace(u[1], v[1], num, endpoint=True)
            for x, y in zip(path_x, path_y):
                if (x, y) != new_path[-1]:
                    new_path.append((x, y))
                    new_path_attr.append(tag)
        else:
            if u != new_path[-1]:
                new_path.append(u)
                new_path_attr.append(tag)
            if v != new_path[-1]:
                new_path.append(v)
                new_path_attr.append(tag)

    return new_path, new_path_attr


def path_concatenation(path1, path2, path1_tags, path2_tags):
    new_path = None
    new_path_tags = None
    if path1 is None:
        new_path = path2
        new_path_tags = path2_tags
    else:
        new_path = path1
        new_path_tags = path1_tags
        if path2 is not None:
            for ii in range(len(path2)):
                p = path2[ii]
                if (p[0], p[1]) != new_path[-1]:
                    new_path.append((p[0], p[1]))
                    new_path_tags.append(path2_tags[ii])

    return new_path, new_path_tags


def is_pathw_valid(pathw: PathWrapper):
    if pathw is None:
        return False
    if pathw.path is None or len(pathw.path) == 0:
        return False
    return True


def pathw_concatenation(pathw1: PathWrapper, pathw2: PathWrapper):
    new_pathw = None
    if pathw1 is None:
        new_pathw = pathw2
    else:
        new_pathw = pathw1
        if pathw2 is not None:
            for ii in range(len(pathw2.path)):
                pps = pathw2.path[ii]
                if pps != new_pathw.path[-1]:
                    tag = pathw2.tags[ii]
                    new_pathw.append_point(pps, tag)

    return new_pathw


def refine_pathw(pathw: PathWrapper, step=0.01):
    if pathw is None or pathw.length() == 0:
        return None

    new_pathw = PathWrapper()
    new_pathw.append_point(pathw.path[0], pathw.tags[0])

    for u, v, tag in zip(pathw.path[:-1], pathw.path[1:], pathw.tags[1:]):
        # print("uv:", u, v)
        num = math.ceil(max(abs(u[0] - v[0]), abs(u[1] - v[1])) / step)
        if num > 1:
            path_x = np.linspace(u[0], v[0], num, endpoint=True)
            path_y = np.linspace(u[1], v[1], num, endpoint=True)
            for x, y in zip(path_x, path_y):
                if (x, y) != new_pathw.path[-1]:
                    new_pathw.append_point((x, y), tag)
        else:
            if u != new_pathw.path[-1]:
                new_pathw.append_point(u, tag)
            if v != new_pathw.path[-1]:
                new_pathw.append_point(v, tag)

    return new_pathw


def nearest_point_of_point_to_path_old(p: Position, path, return_distance=False):
    newp = None
    distance = None
    idx = -1
    for ii in range(len(path)):
        pxy = path[ii]
        dis = euclidean_distance(p, pxy)
        if distance is None:
            distance = dis
            newp = pxy
            idx = ii
        else:
            if distance > dis:
                distance = dis
                newp = pxy
                idx = ii
    if return_distance:
        return newp, idx, distance
    else:
        return newp, idx


def nearest_point_of_point_to_path(p: Position, path, return_distance=False):
    if p is None or path is None or len(path) == 0:
        if return_distance:
            return None, None, None
        else:
            return None, None

    dx = [p[0] - node[0] for node in path]
    dy = [p[1] - node[1] for node in path]

    # x, y = zip(*path)
    # dx = np.array(x) - p[0]
    # dy = np.array(y) - p[1]

    d = np.hypot(dx, dy)
    ind = np.argmin(d)

    if return_distance:
        return path[ind], ind, d[ind]
    else:
        return path[ind], ind

def nearest_point_of_point_to_path_after(p: Position, path, start_idx: int):
    if p is None or path is None or len(path) == 0 or (not start_idx >=0):
        return None, None

    if start_idx > len(path) -1:
        start_idx = len(path) -1

    # dx = [p[0] - node[0] for node in path[start_idx:]]
    # dy = [p[1] - node[1] for node in path[start_idx:]]

    sub_path =  path[start_idx:]
    x, y = zip(*sub_path)
    dx = np.array(x) - p[0]
    dy = np.array(y) - p[1]

    d = np.hypot(dx, dy)
    ind = np.argmin(d)
    ind +=start_idx

    return path[ind], ind
    
import operator


def shrink_path_to_endpoint_old(orin: Position, dest: Position, refined_path, path_attr):
    new_path = []
    new_path_attr = []
    idx_start = 0
    idx_end = len(refined_path)
    if orin is not None:
        for ii in range(len(refined_path)):
            p = refined_path[ii]
            if p == orin:
                idx_start = ii
                # print(idx_start)
                break
    if dest is not None:
        for ii in range(len(refined_path) - 1, -1, -1):
            p = refined_path[ii]
            if p == dest:
                idx_end = ii
                # print(idx_end)
                break

    new_path = refined_path[idx_start : idx_end + 1]
    new_path_attr = path_attr[idx_start : idx_end + 1]
    return new_path, new_path_attr

def shrink_to_endpoint(pathw: PathWrapper, orin: Position, dest: Position):
    idx_start = 0
    idx_end = len(pathw.path) - 1
    if orin is not None:
        _, idx_start = nearest_point_of_point_to_path(orin, pathw.path)
        print("idx_start", idx_start)

    if dest is not None:
        _, idx_end = nearest_point_of_point_to_path(dest, pathw.path)
        print("idx_end", idx_end)

    pathw.shrink_to_endpoint(idx_start,idx_end)

    return pathw


def next_point_extend_distance(point_idx, distance, refined_path):
    distance_sum = 0.0
    for ii in range(point_idx + 1, len(refined_path)):
        distance_sum += euclidean_distance(refined_path[ii - 1], refined_path[ii])
        if distance_sum >= distance:
            return refined_path[ii], ii

    return refined_path[-1], len(refined_path) - 1


def distance_ahead_path(refined_path, start_point_idx, end_point_idx=None) -> float:
    distance_sum = 0.0
    if end_point_idx is None:
        end_point_idx = len(refined_path) - 1
    else:
        end_point_idx = min(end_point_idx, len(refined_path) - 1)
    if start_point_idx >= end_point_idx:
        return 0.0

    for ii in range(start_point_idx + 1, end_point_idx + 1):
        distance_sum += euclidean_distance(refined_path[ii - 1], refined_path[ii])
    return distance_sum


def distance_of_two_point_ahead_path(
    refined_path, orin_idx: int, dest: Position
) -> float:
    distance = None
    if refined_path is not None and orin_idx >= 0 and dest is not None:
        _, new_dest_idx = nearest_point_of_point_to_path_after(dest, refined_path, orin_idx)    

        distance = distance_ahead_path(refined_path, orin_idx, new_dest_idx)
    return distance, new_dest_idx

def calc_path_yaw_np(path):
    if path is None:
        return None

    # x = [p[0] for p in path]
    # y = [p[1] for p in path]

    x, y = zip(*path)  # x, y type is tulple
    x = np.array(x)
    y = np.array(y)
    # print(path)
    # print(x)
    # print(y)

    u = np.gradient(x)
    v = np.gradient(y)
    # tck, myu = splprep([x, y], s=0)
    # u, v = splev(myu, tck, der=1)
    # yaw = list(zip(u, v))
    yaw_angle = np.arctan2(v, u)  # -pi ~ +pi
    # yaw_angle = angle_mod(np.arctan2(v, u), zero_2_2pi=True)
    return yaw_angle


def calc_path_yaw_scipy(path):
    if path is None:
        return None

    ctr = np.array(path)
    x = ctr[:, 0]
    tmin = np.min(x) - 0.01
    tmax = np.max(x) + 0.01
    t = np.linspace(tmin, tmax, len(x))
    spl = make_interp_spline(t, ctr, k=3)
    ders = spl(x, nu=1)
    yaw_angle = np.arctan2(ders[:, 1], ders[:, 0])
    return yaw_angle
