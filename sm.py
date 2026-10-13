import itertools

import numpy as np


def generate_polyline(step=0.02):
    """
    生成具有多个直角拐弯的折线路径。

    参数:
        step: 最大采样间距，单位为米，默认 2 厘米。

    返回:
        shape 为 (N, 2) 的 NumPy 数组。
    """
    if not np.isfinite(step) or step <= 0:
        raise ValueError("step 必须是有限正数")

    # 折线顶点，可以自行修改。
    vertices = np.array(
        [
            [0.0, 0.0],
            [0.6, 0.0],
            [0.6, 0.5],
            [0.7, 0.0],
            [0.8, 0.5],
            [0.6, 0.3],
            [1.2, 0.0],
            [1.8, 0.0],
            [1.8, 0.8],
            [2.4, 0.8],
        ],
        dtype=float,
    )

    segments = []

    for start, end in itertools.pairwise(vertices):
        length = np.linalg.norm(end - start)
        count = int(np.ceil(length / step))

        # 包含起点，不包含终点，避免相邻线段重复添加拐点。
        t = np.arange(count, dtype=float) / count
        segment = start + t[:, None] * (end - start)
        segments.append(segment)

    # 补上整条路径的终点。
    return np.vstack([*segments, vertices[-1:]])


def smooth_path(
    points,
    radius=0.05,
    shape="circle",
    w_ref=1.0,
    w_smooth=20.0,
    fix_ends=True,
    max_iter=10000,
    tol=1e-8,
):
    """
    points: (N, 2)，坐标单位为米。
    radius:
        circle: 圆半径。
        square: 正方形半边长，即 x/y 各允许 ±radius。
    tol: 相邻两轮中，所有点的最大移动量阈值，单位为米。

    返回:
        q: 优化后的点。
        info: 迭代状态。
    """
    p = np.array(points, dtype=float, copy=True)

    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3:
        raise ValueError("points 必须是 (N, 2)，且 N >= 3")
    if not np.all(np.isfinite(p)):
        raise ValueError("坐标必须是有限数值")
    if shape not in ("circle", "square"):
        raise ValueError("shape 必须是 circle 或 square")
    if not np.all(np.isfinite([radius, w_ref, w_smooth, tol])):
        raise ValueError("参数必须是有限数值")
    if radius < 0 or w_ref <= 0 or w_smooth < 0:
        raise ValueError("要求 radius >= 0, w_ref > 0, w_smooth >= 0")
    if max_iter < 1 or tol <= 0:
        raise ValueError("要求 max_iter >= 1, tol > 0")

    q = p.copy()

    # 针对当前目标函数的保守步长。
    step = 1.0 / (2.0 * w_ref + 32.0 * w_smooth)

    for iteration in range(1, max_iter + 1):
        # 1. 贴近原始点的梯度。
        grad = 2.0 * w_ref * (q - p)

        # 2. 二阶差分及其梯度贡献。
        d = q[:-2] - 2.0 * q[1:-1] + q[2:]

        grad[:-2] += 2.0 * w_smooth * d
        grad[1:-1] -= 4.0 * w_smooth * d
        grad[2:] += 2.0 * w_smooth * d

        # 3. 所有点同步进行梯度下降。
        candidate = q - step * grad

        # 4. 相对于原始点进行投影。
        offset = candidate - p

        if shape == "circle":
            length = np.linalg.norm(offset, axis=1)
            outside = length > radius
            offset[outside] *= (radius / length[outside])[:, None]
        else:
            offset = np.clip(offset, -radius, radius)

        q_next = p + offset

        # 5. 固定起终点。
        if fix_ends:
            q_next[0] = p[0]
            q_next[-1] = p[-1]

        max_move = np.max(np.linalg.norm(q_next - q, axis=1))
        q = q_next

        if max_move < tol:
            return q, {
                "converged": True,
                "iterations": iteration,
                "max_step": float(max_move),
            }

    return q, {
        "converged": False,
        "iterations": max_iter,
        "max_step": float(max_move),  # type: ignore
    }


import matplotlib.pyplot as plt

rng = np.random.default_rng(42)

# 生成带有抖动的原始路径。
x = np.linspace(0.0, 2.0, 101)
y = 0.1 * np.sin(3.0 * x)
y += rng.normal(0.0, 0.018, size=x.size)
# points = np.column_stack((x, y))
points = generate_polyline(step=0.02)
optimized, info = smooth_path(
    points,
    radius=0.05,
    shape="circle",  # 改为 "square" 使用正方形约束
    w_ref=1.0,
    w_smooth=20.0,
)

print(info)
print(
    "最大位移 / m:",
    np.linalg.norm(optimized - points, axis=1).max(),
)

plt.plot(*points.T, ".--", alpha=0.5, label="Original")
plt.plot(*optimized.T, ".--", label="Smoothed")
plt.axis("equal")
plt.xlabel("x / m")
plt.ylabel("y / m")
plt.grid(True)
plt.legend()
plt.show()
