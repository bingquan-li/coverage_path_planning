# /usr/bin/env python3
"""SL 横向采样与折线轨迹碰撞检查。

依赖：
    numpy、matplotlib
    Python >= 3.10

运行：
    python sl_sampling_demo.py

无窗口保存：
    python sl_sampling_demo.py --no-show --save sl_sampling_demo.png

说明：
    这里的“可通行”仅表示 SL 平面中的几何无碰撞。
    不包含车辆朝向、曲率、速度和加速度约束。
"""

from dataclasses import dataclass
from itertools import pairwise

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
from numpy.typing import NDArray

# ------------------ 场景参数 ------------------

# 沿参考线方向，每隔 6 m 设置一个采样层。
S_STEP = 6.0

# [0, 6, 12, 18, 24, 30]
# arange 不包含右端点，因此稍微增大终止值以包含 30。
S_SAMPLES = np.arange(0.0, 30.0 + S_STEP / 2, S_STEP)

# 每个采样层的横向候选位置：
# [-3, -2, -1, 0, 1, 2, 3]
L_SAMPLES = np.arange(-2.0, 2.01, 1.0)

ROAD_HALF_WIDTH = 2.6

# 简化机器人包络：始终与 s、l 坐标轴平行的矩形。
# HALF_S 是半长，HALF_L 是半宽，不随轨迹方向旋转。
HALF_S = 0.4
HALF_L = 0.3

# 相邻采样层之间，最多横移 1 m。
MAX_L_CHANGE = 10.0

Vec2 = NDArray[np.float64]


# 曲线离散化时的纵向间隔，与采样层间隔 S_STEP 是两回事。
CURVE_SAMPLE_STEP = 0.1


class QuinticPolynomialCurve1d:
    """五次多项式：

        f(p) = a0 + a1*p + ... + a5*p**5

    start、end：
        [函数值, 一阶导数, 二阶导数]

    param：
        参数域长度，p ∈ [0, param]。

    用于 SL 横向连接时：
        f = l
        p = s - s_start
        param = s_end - s_start

    注意：这里的导数是对 s 求导，不是对时间求导。
    """

    def __init__(self, start, end, param: float):
        self.start_condition = np.asarray(start, dtype=float).copy()
        self.end_condition = np.asarray(end, dtype=float).copy()
        self.param = float(param)

        if self.start_condition.shape != (3,) or self.end_condition.shape != (3,):
            raise ValueError("start 和 end 必须各包含三个元素")

        if not np.isfinite(self.param) or self.param <= 0.0:
            raise ValueError("param 必须是有限的正数")

        if not (
            np.all(np.isfinite(self.start_condition))
            and np.all(np.isfinite(self.end_condition))
        ):
            raise ValueError("起终点条件必须都是有限数值")

        self.coef = self._compute_coefficients()

    def _compute_coefficients(self):
        x0, dx0, ddx0 = self.start_condition
        x1, dx1, ddx1 = self.end_condition

        p = self.param
        p2 = p * p
        p3 = p * p2

        coef = np.zeros(6, dtype=float)

        coef[0] = x0
        coef[1] = dx0
        coef[2] = ddx0 / 2.0

        # 与你提供的 C++ 解析公式一致。
        c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3
        c1 = (dx1 - ddx0 * p - dx0) / p2
        c2 = (ddx1 - ddx0) / p

        coef[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2)
        coef[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p
        coef[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2

        return coef

    def param_length(self) -> float:
        return self.param

    def evaluate(self, order: int, p):
        """计算函数值或导数，支持标量或 NumPy 数组。

        evaluate(0, p)：f(p)
        evaluate(1, p)：f'(p)
        evaluate(2, p)：f''(p)
        """
        if not isinstance(order, (int, np.integer)) or order < 0:
            raise ValueError("order 必须是非负整数")

        values = np.asarray(p, dtype=float)

        if order > 5:
            result = np.zeros_like(values)
        else:
            # 系数按升幂排列：
            # [a0, a1, a2, ...]
            coefficients = self.coef.copy()

            # 每次循环对多项式求一次导：
            # [a0, a1, a2, ...] -> [a1, 2*a2, 3*a3, ...]
            for _ in range(order):
                coefficients = coefficients[1:] * np.arange(1, len(coefficients))

            # 霍纳法求值，避免逐项计算 p 的各次幂。
            result = np.zeros_like(values)
            for coefficient in coefficients[::-1]:
                result = result * values + coefficient

        if values.ndim == 0:
            return float(result)

        return result

    def is_valid(
        self,
        max_value: float = 1.0,
        sample_step: float = 0.1,
    ) -> bool:
        """采样检查 |f(p)| 是否超限，包含起点和终点。

        这是近似检查，不是对连续区间内极值的严格判定。
        不涉及障碍物碰撞。
        """
        if not np.isfinite(max_value) or max_value < 0.0:
            raise ValueError("max_value 必须是有限的非负数")

        if not np.isfinite(sample_step) or sample_step <= 0.0:
            raise ValueError("sample_step 必须是有限的正数")

        count = max(1, int(np.ceil(self.param / sample_step)))
        p = np.linspace(0.0, self.param, count + 1)
        values = self.evaluate(0, p)

        return bool(np.all(np.isfinite(values)) and np.all(np.abs(values) <= max_value))


@dataclass(frozen=True)
class Obstacle:
    """用 SL 坐标系中的轴对齐矩形表示障碍物。

    frozen=True：实例创建后，不能直接修改这些字段。
    """

    s_min: float
    s_max: float
    l_min: float
    l_max: float

    def expanded(self) -> tuple[float, float, float, float]:
        """按机器人的半长、半宽膨胀障碍物。

        对当前固定朝向的矩形模型，可以把：
            机器人矩形与原始障碍物的碰撞
        转换为：
            机器人中心点与膨胀障碍物的碰撞。
        """
        return (
            self.s_min - HALF_S,
            self.s_max + HALF_S,
            self.l_min - HALF_L,
            self.l_max + HALF_L,
        )


OBSTACLES = [
    Obstacle(8.0, 10.5, -0.55, 0.55),
    Obstacle(24.0, 26.5, -1.5, -0.0),
]


@dataclass(frozen=True)
class State:
    s: float
    l: float

    @property
    def sl(self) -> Vec2:
        """只取空间分量。做向量运算时用，**不要**把 t 也塞进欧氏距离里：
        米和秒不同量纲，要混算必须先过 time_scale（见 World.time_scale）。
        """
        return np.array([self.s, self.l], dtype=float)

    def __repr__(self) -> str:
        return f"State({self.s:.3f}, {self.l:.3f})"


# @dataclass(frozen=True) 表示 World 实例的字段不能重新赋值
@dataclass(frozen=True)
class World:
    obstacles: tuple[Obstacle, ...] = ()
    robot_radius: float = 0.2
    horizon: float = 3.0
    max_velocity: float = 3.0
    time_scale: float = 1.0


def connect_nodes(
    start,
    end,
    start_dl: float = 0.0,
    start_ddl: float = 0.0,
    end_dl: float = 0.0,
    end_ddl: float = 0.0,
):
    """用五次多项式连接两个 SL 节点。

    start = (s0, l0)
    end   = (s1, l1)

    返回形状为 (N, 2) 的密集 SL 坐标数组。
    """
    s0, l0 = start
    s1, l1 = end

    delta_s = float(s1 - s0)

    curve = QuinticPolynomialCurve1d(
        start=(l0, start_dl, start_ddl),
        end=(l1, end_dl, end_ddl),
        param=delta_s,
    )

    # ceil 保证实际离散间隔不大于 CURVE_SAMPLE_STEP。
    count = max(
        1,
        int(np.ceil(delta_s / CURVE_SAMPLE_STEP)),
    )

    # p 是局部参数，起点必须是 0。
    p = np.linspace(0.0, delta_s, count + 1)

    s = s0 + p
    l = curve.evaluate(0, p)

    points = np.column_stack((s, l))

    # 消除浮点误差，让拼接端点与原始节点完全一致。
    points[0] = (s0, l0)
    points[-1] = (s1, l1)

    return points


def segment_collision_free(x: State, y: State, world: World) -> bool:
    # for obs in world.obstacles:
    #     box = obs.expanded()
    #     if not box[0] < x.s < box[1]:
    #         continue
    #     if max(x.l, box[2]) < min(y.l, box[3]):
    #         return False
    # return True
    return not any(
        segment_hits_box(x.sl, y.sl, obs.expanded()) for obs in world.obstacles
    )


def uvd_equivalent(
    tau_1: NDArray, tau_2: NDArray, world: World, num_samples: int = 20
) -> bool:
    # sample_s = (tau_1[-1][0] / num_samples) * np.arange(num_samples)
    s_start = max(tau_1[0, 0], tau_2[0, 0])
    s_end = min(tau_1[-1, 0], tau_2[-1, 0])

    if s_end <= s_start:
        return False

    # 在共同区间进行均匀采样
    sample_s = np.linspace(s_start, s_end, num_samples)

    # for idx, point in enumerate(tau_1):
    #     if sample_s[idx] < point[0]:
    #         continue
    #     if not segment_collision_free(
    #         State(point[0], point[1]),
    #         State(tau_2[idx][0], tau_2[idx][1]),
    #         world,
    #     ):
    #         return False

    for s in sample_s:
        l1 = np.interp(s, tau_1[:, 0], tau_1[:, 1])
        l2 = np.interp(s, tau_2[:, 0], tau_2[:, 1])

        if not segment_collision_free(
            State(s, l1),
            State(s, l2),
            world,
        ):
            return False

    return True


# ------------------ 轨迹生成与碰撞检查 ------------------


def segment_hits_box(a, b, box) -> bool:
    """判断线段 ab 是否与闭矩形相交，接触边界也算碰撞。

    线段参数方程：
        p(t) = a + t * (b - a)
        0 <= t <= 1

    分别求线段落在矩形 s 范围、l 范围内的 t 区间。
    两个区间与 [0, 1] 存在公共部分，就表示发生相交。
    """
    enter, leave = 0.0, 1.0

    # box = (s_min, s_max, l_min, l_max)
    # box[::2]  -> (s_min, l_min)
    # box[1::2] -> (s_max, l_max)
    #
    # 循环两次：
    # 第一次处理 s 轴，第二次处理 l 轴。
    for x, dx, low, high in zip(a, b - a, box[::2], box[1::2]):
        if abs(dx) < 1e-12:
            # 线段在当前轴上的坐标不变。
            # 如果这个坐标在矩形范围外，整条线段都不可能相交。
            if x < low or x > high:
                return False
        else:
            # 求经过当前轴两个边界时的参数 t。
            # dx 可能为负，因此排序得到进入和离开的先后顺序。
            t0, t1 = sorted(((low - x) / dx, (high - x) / dx))

            # 与已经得到的有效 t 区间取交集。
            enter = max(enter, t0)
            leave = min(leave, t1)

            if enter > leave:
                return False

    return True


def is_passable(path) -> bool:
    """检查轨迹是否越界，以及是否与障碍物碰撞。"""

    # path 的形状为 (节点数, 2)：
    # path[:, 0] 是全部 s 坐标；
    # path[:, 1] 是全部 l 坐标。
    #
    # 加上机器人半宽后触碰道路边界，也视为不可通行。
    # 对当前直线边界和折线轨迹，节点不越界即可保证线段不越界。
    if np.any(np.abs(path[:, 1]) + HALF_L >= ROAD_HALF_WIDTH):
        return False

    # 任意线段碰到任意膨胀障碍物，都表示整条轨迹不可通行。
    # any() 一旦遇到 True 就停止后续检查。
    return not any(
        segment_hits_box(a, b, obs.expanded())
        for a, b in pairwise(path)
        for obs in OBSTACLES
    )


def generate_candidates():
    """逐层生成路径，连接段碰撞后立即停止该分支。

    返回：
        free_paths：到达最后一层的完整无碰撞轨迹。
        blocked_segments：搜索中遇到的碰撞连接段，用于灰色绘制。

    仍然使用之前的 connect_nodes() 和 is_passable()。
    """
    free_paths = []
    blocked_segments = []

    # key -> (密集采样点, 是否可通行)
    # 同一段连接只生成、检查一次。
    segment_cache = {}

    # 当前搜索分支已经通过的连接段。
    current_segments = []

    def search(layer_index: int, current_l: float):
        # 到达最后一层，保存完整轨迹。
        if layer_index == len(S_SAMPLES) - 1:
            if not current_segments:
                path = np.array(
                    [[S_SAMPLES[0], current_l]],
                    dtype=float,
                )
            else:
                # 第一段保留起点，后续段去掉重复的连接点。
                parts = [current_segments[0]]
                parts.extend(segment[1:] for segment in current_segments[1:])
                path = np.concatenate(parts, axis=0)

            free_paths.append(path)
            return

        s0 = float(S_SAMPLES[layer_index])
        s1 = float(S_SAMPLES[layer_index + 1])

        # 尝试连接下一层的每个横向采样节点。
        for next_l in L_SAMPLES:
            next_l = float(next_l)

            # 先检查便宜的横移约束，再生成曲线。
            if abs(next_l - current_l) > MAX_L_CHANGE + 1e-9:
                continue

            key = (layer_index, current_l, next_l)

            if key not in segment_cache:
                points = connect_nodes(
                    start=(s0, current_l),
                    end=(s1, next_l),
                )

                passable = is_passable(points)
                segment_cache[key] = (points, passable)

                # 每条碰撞连接只记录一次，避免重复绘制。
                if not passable:
                    blocked_segments.append(points)

            points, passable = segment_cache[key]

            if not passable:
                # 核心剪枝：
                # 当前连接发生碰撞，不递归生成它后面的路径。
                continue

            # 当前连接可通行，将它加入路径。
            current_segments.append(points)

            # 从下一层节点继续向后搜索。
            search(layer_index + 1, next_l)

            # 回溯：移除刚才加入的连接，尝试其他 next_l。
            current_segments.pop()

    if len(S_SAMPLES) == 0:
        return free_paths, blocked_segments

    start = np.array([[S_SAMPLES[0], 0.0]], dtype=float)

    if is_passable(start):
        search(layer_index=0, current_l=0.0)

    return free_paths, blocked_segments


def evenly_select(paths, count):
    """按列表索引均匀选取部分轨迹，避免绘图过于密集。

    这里均匀的是索引，不保证轨迹在几何形状上均匀分布。
    """
    if not paths:
        return []

    indices = np.linspace(
        0,
        len(paths) - 1,
        min(count, len(paths)),
        dtype=int,
    )
    return [paths[i] for i in indices]


def distinct_trajectories(
    paths: list[NDArray], world: World, num_samples: int = 20
) -> list[list[NDArray]]:
    kepts: list[list[NDArray]] = []
    for path in paths:
        is_new = True
        for kept in kepts:
            for k1 in kept:
                if uvd_equivalent(k1, path, world, num_samples):
                    is_new = False
                    kept.append(path)
                    break
            if not is_new:
                break
        if is_new:
            # kepts.append(list(path))
            kepts.append([path])
    return kepts


# ------------------ Matplotlib 场景绘制 ------------------


def plot_scene(ax: Axes):
    """在指定的 Axes 上绘制道路、采样点和障碍物。

    Figure：整张画布。
    Axes：画布中的一个绘图区，包含坐标轴、曲线、文字等。
    """

    # 设置绘图区背景色，不是整张 Figure 的背景色。
    ax.set_facecolor("#f8fafc")

    # zorder 控制绘制层级：数值越大，越靠上。
    # 不传 zorder 时，不同类型的图形有各自的默认值。

    # axvline：在指定 x 坐标处画竖线，默认贯穿绘图区高度。
    # 在这里 x 就是 s，每条竖线对应一个采样层。
    for s in S_SAMPLES:
        ax.axvline(
            s,
            color="#dce3eb",
            lw=0.9,  # linewidth：线宽，单位为 point
            zorder=0,
        )

    # axhline：在指定 y 坐标处画横线。
    # ls 是 linestyle 的缩写，"--" 表示虚线。
    ax.axhline(
        0,
        color="#cbd5e1",
        ls="--",
        lw=1,
        zorder=0,
    )

    # 道路左右边界，在 SL 图中表现为两条水平线。
    for l in (-ROAD_HALF_WIDTH, ROAD_HALF_WIDTH):
        ax.axhline(
            l,
            color="#475569",
            lw=2,
            zorder=5,
        )

    # meshgrid 将两组一维坐标展开成二维网格。
    # ss 和 ll 的对应位置组成一个采样点 (s, l)。
    #
    # 跳过第一层，因为第一层只保留固定起点 (0, 0)。
    ss, ll = np.meshgrid(S_SAMPLES[1:], L_SAMPLES)

    # scatter：散点图。
    # 注意这里的参数 s=23 表示标记面积，单位是 point²，
    # 与 SL 坐标中的纵向坐标 s 没有关系。
    ax.scatter(
        ss,
        ll,
        s=23,
        color="#94a3b8",  # 标记填充颜色
        edgecolors="white",  # 标记边缘颜色
        linewidths=0.6,  # 标记边缘线宽
        zorder=4,
    )

    for i, obs in enumerate(OBSTACLES, start=1):
        s_min, s_max, l_min, l_max = obs.expanded()

        # Rectangle((x, y), width, height)：
        # (x, y) 是矩形左下角，宽高使用坐标轴的数据单位。
        #
        # Rectangle 只创建图形对象；
        # add_patch 才把它加入当前绘图区。
        ax.add_patch(
            Rectangle(
                (s_min, l_min),
                s_max - s_min,
                l_max - l_min,
                facecolor="#ef4444",
                alpha=0.10,  # 透明度：0 完全透明，1 完全不透明
                zorder=1,
            )
        )

        # 原始障碍物放在更高层级，覆盖穿过它的轨迹。
        ax.add_patch(
            Rectangle(
                (obs.s_min, obs.l_min),
                obs.s_max - obs.s_min,
                obs.l_max - obs.l_min,
                facecolor="#334155",
                zorder=6,
            )
        )

        # text(x, y, text)：在数据坐标 (x, y) 处写文字。
        # ha / va 分别控制水平、垂直对齐方式。
        ax.text(
            (obs.s_min + obs.s_max) / 2,
            (obs.l_min + obs.l_max) / 2,
            f"O{i}",
            ha="center",
            va="center",
            color="white",
            fontsize=10,
            zorder=7,
        )

    # 用星形标记起点。
    ax.scatter(
        [0],
        [0],
        s=125,
        marker="*",
        color="#0f172a",
        zorder=9,
    )

    # set 可以一次设置多个属性。
    # 根据 S_SAMPLES 设置横轴范围，调整采样长度后不必另改范围。
    ax.set(
        xlim=(S_SAMPLES[0] - 0.7, S_SAMPLES[-1] + 0.7),
        ylim=(-ROAD_HALF_WIDTH - 0.4, ROAD_HALF_WIDTH + 0.4),
        xlabel="s [m]",
        ylabel="l [m]",
    )

    # 指定刻度位置；不指定时，Matplotlib 会自动选择。
    ax.set_xticks(S_SAMPLES)
    ax.set_yticks(L_SAMPLES)

    # spines 是绘图区四周的边框线。
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # 当前使用默认的自动纵横比，方便观察横向变化。
    # 若希望 s、l 方向的 1 m 显示为相同长度，可启用：
    # ax.set_aspect("equal", adjustable="box")
    ax.set_aspect("equal", adjustable="box")


def main():
    free, blocked = generate_candidates()
    world = World(obstacles=tuple(OBSTACLES))
    uvd_paths = distinct_trajectories(free, world)

    group_count = len(uvd_paths)

    # 两个窗口共享同一组颜色。
    cmap = plt.get_cmap("tab10")
    colors = [cmap(i % 10) for i in range(group_count)]

    def draw_group(ax, group, color):
        """绘制一个类别中的全部轨迹。"""
        for path in group:
            ax.plot(
                path[:, 0],
                path[:, 1],
                color=color,
                linewidth=1.2,
                alpha=0.45,
                zorder=3,
            )

    def group_handle(group_id, group):
        """图例线条独立设置，避免受到轨迹透明度影响。"""
        return Line2D(
            [],
            [],
            color=colors[group_id],
            lw=3,
            label=f"Homotopy {group_id + 1} ({len(group)} paths)",
        )

    scene_handles = [
        Line2D(
            [],
            [],
            color="#a3a8b0",
            lw=1.5,
            label="Blocked connection",
        ),
        Line2D(
            [],
            [],
            marker="o",
            linestyle="none",
            color="#94a3b8",
            label="Lateral sample",
        ),
        Patch(
            facecolor="#334155",
            label="Obstacle",
        ),
        Patch(
            facecolor="#ef4444",
            alpha=0.10,
            label="Inflated obstacle",
        ),
    ]

    # ------------------ 窗口 1：所有类别 ------------------

    fig_all, ax_all = plt.subplots(
        figsize=(14, 5),
        dpi=150,
        layout="constrained",
    )
    if fig_all.canvas.manager is not None:
        fig_all.canvas.manager.set_window_title("SL Homotopy - All classes")

    fig_all.suptitle(
        "SL lattice sampling — Homotopy classes",
        fontsize=17,
        fontweight="bold",
    )

    plot_scene(ax_all)

    for segment in blocked:
        ax_all.plot(
            segment[:, 0],
            segment[:, 1],
            color="#a3a8b0",
            linewidth=1.0,
            alpha=0.4,
            zorder=2,
        )

    for group_id, group in enumerate(uvd_paths):
        draw_group(ax_all, group, colors[group_id])

    ax_all.set_title(
        f"{len(free)} collision-free paths | "
        f"{group_count} Homotopy classes | "
        f"{len(blocked)} blocked connections",
        loc="left",
        fontsize=11,
    )

    # 将每个类别的颜色说明加入图例。
    class_handles = [
        group_handle(group_id, group) for group_id, group in enumerate(uvd_paths)
    ]

    ax_all.legend(
        handles=class_handles + scene_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.25),
        ncol=min(4, len(class_handles + scene_handles)),
        frameon=False,
        fontsize=9,
    )

    # ------------------ 窗口 2：每个类别单独显示 ------------------

    # 没有可行类别时，也创建一个子图说明情况。
    rows = max(1, group_count)

    fig_groups, axes = plt.subplots(
        nrows=rows,
        ncols=1,
        figsize=(14, 3.2 * rows),
        dpi=150,
        squeeze=False,
        layout="constrained",
    )
    if fig_groups.canvas.manager is not None:
        fig_groups.canvas.manager.set_window_title("SL Homotopy - Individual classes")

    fig_groups.suptitle(
        "Individual Homotopy classes",
        fontsize=17,
        fontweight="bold",
    )

    if group_count == 0:
        ax = axes[0, 0]
        plot_scene(ax)
        ax.set_title("No collision-free paths", loc="left")
    else:
        for group_id, group in enumerate(uvd_paths):
            ax = axes[group_id, 0]
            color = colors[group_id]

            plot_scene(ax)
            draw_group(ax, group, color)

            ax.set_title(
                f"Homotopy class {group_id + 1} — {len(group)} paths",
                color=color,
                fontsize=12,
                fontweight="bold",
                loc="left",
            )

            ax.legend(
                handles=[group_handle(group_id, group)],
                loc="upper right",
                frameon=True,
                facecolor="white",
                framealpha=0.9,
                fontsize=9,
            )

    # ------------------ 保存与显示 ------------------

    print(f"Homotopy 类别数：{group_count}")
    for group_id, group in enumerate(uvd_paths, start=1):
        print(f"  类别 {group_id}：{len(group)} 条轨迹")

    # 一次 show() 同时显示两个 Figure 窗口。
    plt.show()

    plt.close(fig_all)
    plt.close(fig_groups)


if __name__ == "__main__":
    main()
