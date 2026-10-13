#!/usr/bin/env python3
"""固定半宽的法向路径带：圆弧倒角 + 局部曲率约束 + 离散条带碰撞检查。

安装：python -m pip install numpy matplotlib
运行：python normal_band_demo.py
只保存图片：python normal_band_demo.py --no-show

坐标单位可理解为米。仅支持开放折线，自动去除连续重复点。
圆弧连接保证切线连续，曲率可在直线/圆弧接点处跳变。
全局检测针对生成的离散四边形，使用浮点容差；不是连续曲线的严格证明。
示例不包含障碍物约束，也不自动搜索能够消除全局冲突的新路线。
"""

import argparse
from pathlib import Path

import numpy as np


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def left_normal(t):
    return np.stack((-t[..., 1], t[..., 0]), axis=-1)


def clean_points(points):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or not np.isfinite(p).all():
        raise ValueError("points 必须为有限的 N×2 数组")
    if len(p) < 2:
        raise ValueError("至少需要两个不同点")
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-10]]
    if len(p) < 2:
        raise ValueError("至少需要两个不同点")
    return p


def rounded_polyline(points, radius, ds=0.08):
    """用指定半径的相切圆弧替换每个折线拐角。

    返回采样点、解析单位切线、整条连续路径的最大绝对曲率。
    无空间放入倒角或遇到180度折返时直接失败，不偷偷减小半径。
    """
    if not np.isfinite([radius, ds]).all() or radius <= 0 or ds <= 0:
        raise ValueError("radius 和 ds 必须为有限正数")
    p = clean_points(points)
    delta = np.diff(p, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    directions = delta / lengths[:, None]
    n = len(p)
    angle = np.zeros(n)
    trim = np.zeros(n)
    for i in range(1, n - 1):
        a, b = directions[i - 1], directions[i]
        angle[i] = np.arctan2(cross(a, b), np.dot(a, b))
        if abs(angle[i]) > np.pi - 1e-6:
            raise ValueError(f"顶点 {i} 接近180度折返，需重新规划中心线")
        if abs(angle[i]) < 1e-10:
            angle[i] = 0.0
        trim[i] = radius * np.tan(abs(angle[i]) / 2)
    if np.any(trim[:-1] + trim[1:] >= lengths - 1e-10):
        raise ValueError("线段太短，放不下指定半径的圆弧；需调整中心线")

    samples, tangents = [], []

    def append(ps, ts):
        # 相切连接处只保存一个点，避免零长度单元。
        for q, t in zip(ps, ts):
            if samples and np.linalg.norm(q - samples[-1]) < 1e-10:
                continue
            samples.append(q)
            tangents.append(t)

    def line(start, end, direction):
        length = np.linalg.norm(end - start)
        count = max(1, int(np.ceil(length / ds)))
        q = start + np.linspace(0, 1, count + 1)[:, None] * (end - start)
        append(q, np.tile(direction, (len(q), 1)))

    current = p[0]
    for i in range(1, n - 1):
        incoming, outgoing = directions[i - 1], directions[i]
        entry = p[i] - trim[i] * incoming
        exit_point = p[i] + trim[i] * outgoing
        line(current, entry, incoming)
        if angle[i] != 0:
            sign = np.sign(angle[i])
            center = entry + sign * radius * left_normal(incoming)
            theta0 = np.arctan2(*(entry - center)[::-1])
            count = max(2, int(np.ceil(radius * abs(angle[i]) / ds)))
            theta = theta0 + np.linspace(0, angle[i], count + 1)
            radial = np.column_stack((np.cos(theta), np.sin(theta)))
            append(center + radius * radial, sign * left_normal(radial))
        current = exit_point
    line(current, p[-1], directions[-1])
    kmax = 1 / radius if np.any(angle != 0) else 0.0
    return np.array(samples), np.array(tangents), kmax


def naive_polyline(points, ds=0.08):
    """反例：折线直接采样，顶点用相邻方向平均来猜法线。"""
    p = clean_points(points)
    chunks = []
    for a, b in zip(p[:-1], p[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / ds)))
        chunks.append(a + np.arange(count)[:, None] / count * (b - a))
    q = np.vstack([*chunks, p[-1:]])
    tangent = np.gradient(q, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-10):
        raise ValueError("平均切线退化")
    return q, tangent / norm[:, None]


def sat_overlap(a, b, allow_shared_boundary, tol):
    """凸多边形分离轴检测；相邻单元可共享边，但不可内部重叠。"""
    edges = np.vstack((np.roll(a, -1, axis=0) - a, np.roll(b, -1, axis=0) - b))
    axes = left_normal(edges)
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    pa, pb = a @ axes.T, b @ axes.T
    depth = np.minimum(pa.max(axis=0), pb.max(axis=0)) - np.maximum(
        pa.min(axis=0), pb.min(axis=0)
    )
    # 相邻单元只要有一条轴的重叠为0，就是合法公共边。
    # 不相邻单元的接触也按冲突处理，容差内的近接触保守拒绝。
    return bool(np.all(depth > tol) if allow_shared_boundary else np.all(depth >= -tol))


def validate_band(p, tangent, width, tol=1e-9):
    """检查开放路径的离散法向条带，时间复杂度 O(N²)。

    对每个四边形要求严格凸且逆时针，从而拒绝局部翻转/退化。
    然后使用包围盒筛选和SAT检测，包括相邻单元的内部重叠。
    返回左右边界、四边形、坏单元索引及冲突单元对。
    """
    if not np.isfinite(width) or width <= 0:
        raise ValueError("width 必须为有限正数")
    p, tangent = np.asarray(p, float), np.asarray(tangent, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or tangent.shape != p.shape:
        raise ValueError("p 和 tangent 必须为相同形状的 N×2 数组，N>=2")
    if not np.isfinite(p).all() or not np.isfinite(tangent).all():
        raise ValueError("输入包含非有限值")
    if not np.allclose(np.linalg.norm(tangent, axis=1), 1, atol=1e-8, rtol=0):
        raise ValueError("tangent 必须为单位切线")
    normal = left_normal(tangent)
    left, right = p + width * normal, p - width * normal
    quads = np.stack((left[:-1], right[:-1], right[1:], left[1:]), axis=1)
    edges = np.roll(quads, -1, axis=1) - quads
    edge_lengths = np.linalg.norm(edges, axis=2)
    # cross/边长 的量纲为长度，与 tol 保持一致。
    heights = cross(edges, np.roll(edges, -1, axis=1)) / np.maximum(edge_lengths, tol)
    valid = np.all(heights > tol, axis=1) & np.all(edge_lengths > tol, axis=1)
    bad_cells = np.flatnonzero(~valid).tolist()
    lows, highs = quads.min(axis=1), quads.max(axis=1)
    collisions = []
    for i in range(len(quads)):
        if not valid[i]:
            continue
        for j in range(i + 1, len(quads)):
            if not valid[j]:
                continue  # 已被局部检查判为失败。
            if np.any(highs[i] < lows[j] - tol) or np.any(highs[j] < lows[i] - tol):
                continue
            if sat_overlap(quads[i], quads[j], j == i + 1, tol):
                collisions.append((i, j))
    return dict(
        left=left,
        right=right,
        quads=quads,
        bad_cells=bad_cells,
        collisions=collisions,
        ok=not bad_cells and not collisions,
    )


def build_safe_band(points, width, eta=0.8, preferred_radius=None, ds=0.08):
    """固定宽度：选择足够大的倒角半径，失败时明确拒绝。

    这不是任意路线的修复器。半径无法放入或全局冲突时交给上层调整路线。
    返回 p, tangent, report；left[i]/right[i] 与 p[i] 一一对应。
    """
    if (
        not np.isfinite([width, eta, ds]).all()
        or width <= 0
        or ds <= 0
        or not 0 < eta < 1
    ):
        raise ValueError("要求 width>0、ds>0、0<eta<1，且均为有限值")
    if preferred_radius is not None and (
        not np.isfinite(preferred_radius) or preferred_radius <= 0
    ):
        raise ValueError("preferred_radius 必须为有限正数")
    radius = max(width / eta, preferred_radius or 0.0)
    p, tangent, kmax = rounded_polyline(points, radius, ds)
    if width * kmax > eta + 1e-12:
        raise ValueError("曲率约束未通过")
    report = validate_band(p, tangent, width)
    report.update(radius=radius, max_abs_curvature=kmax)
    if not report["ok"]:
        raise ValueError(
            f"条带检查失败：{len(report['bad_cells'])} 个坏单元，"
            f"{len(report['collisions'])} 对冲突单元；需调整中心线"
        )
    return p, tangent, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--output", default="normal_band_demo.png")
    args = parser.parse_args()
    import matplotlib

    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection

    width, eta, ds = 0.45, 0.8, 0.08
    corner = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 3.0]])
    returning = np.array(
        [[0.0, 0.0], [5.0, 0.0], [5.0, 4.0], [-2.0, 4.0], [-2.0, 0.65], [3.0, 0.65]]
    )
    p0, t0 = naive_polyline(corner, ds)
    p1, t1, k1 = rounded_polyline(corner, 0.22, ds)
    p2, t2, safe = build_safe_band(corner, width, eta, preferred_radius=1.0, ds=ds)
    p3, t3, k3 = rounded_polyline(returning, 1.0, ds)
    cases = [
        ("A  Sharp corner / averaged normal", corner, p0, t0, np.inf),
        ("B  Rounded, but radius too small", corner, p1, t1, k1),
        (
            "C  Sufficient radius + global check",
            corner,
            p2,
            t2,
            safe["max_abs_curvature"],
        ),
        ("D  Local curvature OK / global overlap", returning, p3, t3, k3),
    ]
    plt.rcParams.update(
        {"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}
    )
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), layout="constrained")
    fig.suptitle(
        "Fixed-width normal band: smooth, offset, validate\n"
        f"Half-width w = {width:.2f} m  |  curvature margin eta = {eta}",
        fontsize=17,
    )
    results = []
    for ax, (title, original, p, tangent, kmax) in zip(axes.flat, cases):
        report = validate_band(p, tangent, width)
        local_ok = width * kmax <= eta + 1e-12
        accepted = local_ok and report["ok"]
        results.append(accepted)
        red = set(report["bad_cells"])
        for i, j in report["collisions"]:
            red.update((i, j))
        colors = [
            "#ed6a5a" if i in red else "#9dd9d2" for i in range(len(report["quads"]))
        ]
        ax.add_collection(
            PolyCollection(
                report["quads"], facecolors=colors, alpha=0.48, edgecolors="none"
            )
        )
        ax.plot(*original.T, ":", c="#9b9b9b", lw=1.5, label="Original polyline")
        ax.plot(*p.T, c="#233647", lw=2.0, label="Centerline")
        ax.plot(*report["left"].T, c="#147d92", lw=1.5, label="Left boundary")
        ax.plot(*report["right"].T, c="#c87817", lw=1.5, label="Right boundary")
        stride = max(1, int(0.23 / ds))
        sections = np.stack(
            (report["left"][::stride], report["right"][::stride]), axis=1
        )
        ax.add_collection(
            LineCollection(sections, colors="#596e7c", linewidths=0.7, alpha=0.55)
        )
        curvature_text = (
            "undefined at corner"
            if not np.isfinite(kmax)
            else f"w * max|k| = {width * kmax:.2f}"
        )
        status = "PASS" if accepted else "REJECT"
        ax.set_title(
            f"{title}\n{status}  |  {curvature_text}",
            loc="left",
            fontsize=11,
            color="#126e57" if accepted else "#b63e32",
            fontweight="bold",
        )
        ax.text(
            0.02,
            0.97,
            f"Invalid cells: {len(report['bad_cells'])}\n"
            f"Collision pairs: {len(report['collisions'])}",
            transform=ax.transAxes,
            va="top",
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.88, edgecolor="none"),
        )
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(alpha=0.15)
        ax.margins(0.14)
        print(
            f"{title}: {status}; {curvature_text}; "
            f"bad_cells={len(report['bad_cells'])}, collision_pairs={len(report['collisions'])}"
        )
    assert results == [False, False, True, False], results
    # 保证安全生成函数会拒绝“曲率合格但全局重叠”的路线。
    try:
        build_safe_band(returning, width, eta, preferred_radius=1.0, ds=ds)
    except ValueError as exc:
        print("Global-conflict rejection verified:", exc)
    else:
        raise AssertionError("全局冲突未被拒绝")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=4, frameon=False)
    target = Path(args.output).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(target, dpi=170, facecolor="white")
    print("Saved:", target)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
