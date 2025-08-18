from .protocols import Position
from .node_attribute import NodeAttribute


class PointWrapper:
    point: Position
    tag: NodeAttribute
    yaw: float

    def __init__(self, point: Position, tag: NodeAttribute):
        self.point = point
        self.tag = tag
        self.yaw = 0
