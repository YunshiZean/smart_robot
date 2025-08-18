import numpy as np

from .protocols import Position
from .node_attribute import NodeAttribute


class PathWrapper:
    path: list
    tags: list
    yaws: list

    def __init__(self):
        self.path = []
        self.tags = []
        self.yaws = []    

    def shrink_to_endpoint(self,idx_start, idx_end):
        if idx_end > idx_start: # at least two point
            self.path = self.path[idx_start : idx_end + 1]
            self.tags = self.tags[idx_start : idx_end + 1]
        else:
            self.clear()

    def append_point(self, point: Position, tag: NodeAttribute):
        self.path.append(point)
        self.tags.append(tag)
        # self.yaws.append(yaw)

    def get_tag_value(self, point_idx, tag_key):
        if point_idx is not None and point_idx >= 0 and point_idx < self.length():
            tag: NodeAttribute = self.tags[point_idx]
            return tag.get_tag_value(tag_key)
        return None

    def clear(self):
        self.path = []
        self.tags = []
        self.yaws = []

    def length(self):
        if self.path is None:
            return 0
        return len(self.path)

    def calc_yaw_np(self):
        if self.length() == 0:
            self.yaws = []
        else:
            x, y = zip(*self.path)  # x, y type is tulple

            x = np.array(x)
            y = np.array(y)
            # print(pathw)
            # print(x)
            # print(y)

            u = np.gradient(x)
            v = np.gradient(y)
            self.yaws = np.arctan2(v, u).tolist()  # -pi ~ +pi

    def __str__(self):
        return str(self.path) + str(self.tags)

    def __repr__(self):
        return str(self.path) + str(self.tags)
