from shapely import LineString, offset_curve
import matplotlib.pyplot as plt
import numpy as np
import math


def test1():
    cx = np.arange(0, 50, 0.01)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    line = LineString([(xy[0], xy[1]) for xy in zip(cx, cy)])
    offsetline = offset_curve(line, 0.3)
    offsetline2 = offset_curve(line, -0.3)

    print(offsetline.geom_type)

    # lens were different
    print(len(line.coords))
    print(len(offsetline.coords))
    print(len(offsetline2.coords))

    x, y = line.xy
    plt.plot(x, y)
    x, y = offsetline.xy
    # plt.plot(x,y)
    # x,y=offsetline2.xy
    plt.plot(x, y)
    plt.show()



from numpy import asarray
def test2():
    cx = [0, 1]
    cy = [2, 3]

    line = LineString([(xy[0], xy[1]) for xy in zip(cx, cy)])
    offsetline = offset_curve(line, 0.3)

    # 1.dont work
    # points = np.array(offsetline)
    # for p in points:
    #     print(p)
    # 2.dont work
    # for x,y in offsetline.exterior.coords:
    #     print(x,y)  
    # 3. dont work     
    # print(asarray(offsetline))
    # for p in asarray(offsetline):
    #     print(p)

    # 3. ok
    # for x,y in offsetline.coords:
    #     print(x,y) 
    # 4. ok
    # cx, cy = offsetline.xy
    # for x,y in zip(cx,cy):
    #     print(x,y)

    detect_area_lines = []
    detect_area_lines.extend(offsetline.coords)
    print(detect_area_lines)

if __name__ == "__main__":
    test1()
