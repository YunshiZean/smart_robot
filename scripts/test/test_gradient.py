import numpy as np
import matplotlib.pyplot as plt
import sys
from scipy.interpolate import splprep, splev, splrep
from scipy.interpolate import make_interp_spline

sys.path.append("..")

path = [
    (0.37320464524060387, -0.06263250302462511),
    (0.36, -0.06263250302462511),
    (0.383423735511064, -0.05468432170315612),
    (0.39364282578152415, -0.04673614038168712),
    (0.4038619160519843, -0.03878795906021813),
    (0.4140810063224445, -0.030839777738749144),
    (0.42430009659290463, -0.022891596417280148),
    (0.43451918686336477, -0.014943415095811152),
    (0.4447382771338249, -0.006995233774342163),
    (0.45495736740428505, 0.0009529475471268256),
    (0.4657005135860508, 0.0014770034584324996),
    (0.4764436597678166, 0.0020010593697381736),
    (0.48718680594958236, 0.002525115281043847),
    (0.4979299521313481, 0.003049171192349521),
    (0.5086730983131139, 0.003573227103655195),
    (0.5194162444948797, 0.004097283014960869),
    (0.5301593906766454, 0.004621338926266543),
    (0.5409025368584112, 0.005145394837572217),
    (0.5516456830401769, 0.005669450748877891),
    (0.5623888292219428, 0.006193506660183565),
    (0.5731319754037085, 0.006717562571489239),
    (0.5838751215854743, 0.007241618482794912),
    (0.59461826776724, 0.007765674394100586),
    (0.6046965992185478, 0.007793826157931056),
    (0.6147749306698556, 0.007821977921761526),
    (0.6248532621211633, 0.007850129685591995),
    (0.6349315935724711, 0.007878281449422465),
    (0.6450099250237789, 0.007906433213252935),
    (0.6550882564750867, 0.007934584977083407),
    (0.6651665879263945, 0.007962736740913877),
    (0.6752449193777023, 0.007990888504744347),
    (0.68532325082901, 0.008019040268574817),
    (0.6954015822803178, 0.008047192032405287),
    (0.7054799137316256, 0.008075343796235757),
    (0.7155582451829333, 0.008103495560066227),
    (0.7256365766342411, 0.008131647323896696),
    (0.7357149080855488, 0.008159799087727166),
    (0.7457932395368567, 0.008187950851557636),
    (0.7558715709881645, 0.008216102615388106),
    (0.7659499024394723, 0.008244254379218578),
    (0.77602823389078, 0.008272406143049048),
    (0.7861065653420878, 0.008300557906879518),
    (1.17961848967933955, 0.008328709670709988),
]


# https://blog.csdn.net/weixin_42694889/article/details/117222471
def test1():
    t = np.arange(0, 1, 0.1)
    t = np.insert(t, 0, t[0])
    t = np.insert(t, 0, t[0])
    t = np.insert(t, 0, t[0])

    y = t[::-1]
    y = t

    # print(t)
    # print(y)
    u = np.gradient(t)
    v = np.gradient(y)

    axes = plt.subplot(1, 2, 1, aspect="equal")
    # ax.set_aspect(1)
    axes.scatter(t, y)
    axes.quiver(t, y, u, v)

    y = t**2 - 2 * t + 1
    u = np.gradient(t)
    v = np.gradient(y)
    axes.scatter(t, y)
    axes.quiver(t, y, u, v)

    # print(t)
    # print(y)
    # print(u)
    # print(v)

    # plt.show()

    # y = t[::-1]

    # t = [0.22673102, 0.22673102, 0.23721214, 0.24769325, 0.25817437, 0.26865549]
    # y = [-0.15573977, -0.15573977, -0.14918907, -0.14263837, -0.13608767, -0.12953697]
    # tck = splrep(t, y, s=0)
    # u = splev(t, tck, der=1)
    # print("tck", tck)
    # print(u)

    ctr = np.array(path)
    x = ctr[:, 0]
    y = ctr[:, 1]

    t = np.linspace(0, 1, len(x))

    tck, myu = splprep([t, x, y], s=0)
    print(tck)
    print(myu)

    tck, myu = splprep([t, x], s=0)
    print(tck)
    print(myu)
    # u, v = splev(myu, tck, der=1)

    # axes = plt.subplot(1, 2, 2, aspect="equal")
    # axes.scatter(t, y)
    # axes.quiver(t, y, u, v)

    # y = t**2 - 2 * t + 1
    # tck, myu = splprep([t, y], s=0)
    # u, v = splev(myu, tck, der=1)

    # axes.scatter(t, y)
    # axes.quiver(t, y, t, v)

    # plt.tight_layout()
    # plt.show()


def test2():
    t = np.arange(0, 2, 0.1)
    # t = np.arange(0, 0.2, 0.1)
    y = t**2 - 2 * t + 1
    # y=t
    from pyadroute.utils.path import calc_path_yaw

    path = [(p[0], p[1]) for p in zip(t, y)]
    yaw_angle, path_yaw = calc_path_yaw(path)

    print(path)
    print(path_yaw)
    print(yaw_angle * 180 / np.pi)

    # u = [p[0] for p in path_yaw]
    # v = [p[1] for p in path_yaw]

    u, v = zip(*path_yaw)

    ax = plt.gca()  # 获取当前的Axes对象
    ax.set_aspect(1)  # 设置x,y轴等比例
    plt.scatter(t, y)
    plt.quiver(t, y, u, v)
    plt.tight_layout()
    plt.show()


def test3():

    x, y = zip(*path)  # x, y type is tulple
    x = np.array(x)
    y = np.array(y)

    ax = plt.gca()  # 获取当前的Axes对象
    ax.set_aspect(1)  # 设置x,y轴等比例
    plt.scatter(x, y)

    u = np.gradient(x)
    v = np.gradient(y)

    # 执行3阶多项式拟合
    coefficients = np.polyfit(x, y, 3)

    # 创建多项式对象
    p = np.poly1d(coefficients)

    # 打印多项式方程
    print(p)
    y = p(x)

    plt.scatter(x, y)

    # plt.quiver(x, y, u, v)

    plt.show()


def test4():
    from geomdl import fitting
    from geomdl.visualization import VisMPL as vis

    points = path  # x, y type is tulple
    # points = [(0, 0), (3, 4), (-1, 4), (-4, 0), (-4, -3)]

    x, y = zip(*points)  # x, y type is tulple
    x = np.array(x)
    y = np.array(y)
    degree = 3  # cubic curve

    # Do global curve interpolation
    curve = fitting.interpolate_curve(points, degree)
    # curve = fitting.approximate_curve(points, degree)
    ders = []
    curve.delta = 0.001
    for p in points:
        der = curve.derivatives(p[0], order=1)
        ders.append(der[1])
    print(ders)

    u = [der[0] for der in ders]
    v = [der[1] for der in ders]

    evalpts = np.array(curve.evalpts)
    pts = np.array(points)

    # Plot points together on the same graph
    fig = plt.figure(figsize=(10, 8), dpi=96)
    plt.plot(evalpts[:, 0], evalpts[:, 1])
    plt.scatter(pts[:, 0], pts[:, 1], color="red")
    plt.quiver(x, y, u, v)
    # plt.scatter(x, y)
    plt.show()


def test5():
    from geomdl import fitting
    from geomdl.visualization import VisMPL as vis
    import numpy as np
    import matplotlib.pyplot as plt

    # The NURBS Book Ex9.1
    points = ((0, 0), (3, 4), (-1, 4), (-4, 0), (-4, -3))
    degree = 3  # cubic curve
    points = path
    # Do global curve interpolation
    curve = fitting.interpolate_curve(points, degree)

    # Prepare points
    evalpts = np.array(curve.evalpts)
    pts = np.array(points)

    # Plot points together on the same graph
    fig = plt.figure(figsize=(10, 8), dpi=96)
    plt.plot(evalpts[:, 0], evalpts[:, 1])
    plt.scatter(pts[:, 0], pts[:, 1], color="red")
    plt.show()


def test6():

    ax = plt.gca()  # 获取当前的Axes对象
    ax.set_aspect(1)  # 设置x,y轴等比例

    path = [
        [1, 0.01],
        [1, 0.02],
        [1, 0.03],
        [1, 0.04],
        [1, 0.05],
        [1, 0.06],
        [1, 0.07],
        [1, 0.08],
        [1, 0.09],
        [1, 0.10],
    ]
    # 定义控制点
    ctr = np.array(path)
    x = ctr[:, 0]
    y = ctr[:, 1]
    tmin = np.min(x) - 0.01
    tmax = np.max(x) + 0.01
    t = np.linspace(tmin, tmax, len(x))
    spl = make_interp_spline(t, ctr, k=3)
    x2 = np.linspace(tmin, tmax, 100)
    x2 = x
    y2 = spl(x2)
    ders = spl(x2, nu=1)
    print(np.arctan2(ders[:, 1], ders[:, 0]))
    print(len(x2))
    print(len(y2))

    u = [der[0] for der in ders]
    v = [der[1] for der in ders]
    print(len(u))
    print(len(v))
    # 绘制结果
    plt.scatter(x, y, color="red", label="Control Points")
    plt.scatter(y2[:, 0], y2[:, 1], color="red", label="Control Points")
    # plt.plot(curve[:, 0], curve[:, 1], "b-", label="B-spline Curve")
    plt.quiver(y2[:, 0], y2[:, 1], u, v)
    plt.legend(loc="best")
    plt.title("B-Spline Interpolation")
    plt.show()


if __name__ == "__main__":
    test6()
