from shapely import LineString, offset_curve


# bounds: numpy ndarray of [xmin, ymin, xmax, ymax]
def point_in_bounds(p, bounds):
    if (
        p[0] >= bounds[0]
        and p[0] <= bounds[2]
        and p[1] >= bounds[1]
        and p[1] <= bounds[3]
    ):
        return True
    return False
 
#   |---------|    -
# p1|         |p2  2*offset
#   |---------|    -
def get_rectangle(p1, p2, offset):
    line1 = LineString([p1, p2])
    left_line = offset_curve(line1, offset).coords
    right_line = offset_curve(line1, -offset).coords
    return left_line[0], left_line[-1], right_line[-1], right_line[0]
