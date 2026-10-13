"""Fit two locally straight corridor walls from uninflated grid cells.

Uses class 2 by default, never treats assumed-free map padding as evidence.
Coordinates are fitted estimates, not certified physical wall surfaces or a
vehicle collision check. Curved walls and junctions need a different model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from astar_frame import estimate_robot_pose


def obstacle_points(grid, obstacle_class=2):
    cells = np.asarray(grid["data"])
    resolution = float(grid["map_scale"])
    if cells.shape != (grid["height"], grid["width"]):
        raise ValueError("地图尺寸不匹配。")
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("无效的地图分辨率。")
    row_order = grid.get("row_order", "bottom_to_top")
    if row_order == "top_to_bottom":
        cells = cells[::-1]
    elif row_order != "bottom_to_top":
        raise ValueError("未知的地图行序。")
    origin = np.asarray(grid["origin"], dtype=float)
    q = np.asarray(grid.get("rotation_xyzw", [0, 0, 0, 1]), dtype=float)
    if (origin.shape != (2,) or not np.isfinite(origin).all()
            or q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) == 0):
        raise ValueError("无效的地图位姿。")
    q /= np.linalg.norm(q)
    if not np.allclose(q[:2], 0):
        raise ValueError("只支持二维地图。")
    yaw = 2 * np.arctan2(q[2], q[3])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rows, cols = np.where(cells == obstacle_class)
    return np.column_stack([cols + 0.5, rows + 0.5]) * resolution @ rotation.T + origin


def fit_side(points, tolerance=0.08, min_length=1.0):
    """RANSAC l=a*s+b in the vehicle frame, then total least squares.

    Reject short blobs and lines more than 20 degrees from the vehicle heading.
    Longitudinal coverage rejects two isolated clusters masquerading as a wall.
    """
    if len(points) < 20:
        return None
    rng = np.random.default_rng(7)
    best, best_score = None, -1
    for _ in range(1200):
        p, q = points[rng.choice(len(points), 2, replace=False)]
        if abs(q[0] - p[0]) < min_length / 2:
            continue
        a = (q[1] - p[1]) / (q[0] - p[0])
        if abs(a) > np.tan(np.deg2rad(20)):
            continue
        b = p[1] - a * p[0]
        mask = np.abs(points[:, 1] - a * points[:, 0] - b) / np.hypot(1, a) <= tolerance
        s = points[mask, 0]
        if len(s) < 20 or np.ptp(s) < min_length:
            continue
        # Favor support across the wall, not a densely sampled small obstacle.
        score = len(np.unique(np.floor(s / 0.1))) * 10 + len(s)
        if score > best_score:
            best, best_score = mask, score
    if best is None:
        return None
    for _ in range(3):
        selected = points[best]
        center = selected.mean(axis=0)
        _, _, vt = np.linalg.svd(selected - center, full_matrices=False)
        direction = vt[0]
        if abs(direction[0]) < 1e-8:
            return None
        a = direction[1] / direction[0]
        b = center[1] - a * center[0]
        best = np.abs(points[:, 1] - a * points[:, 0] - b) / np.hypot(1, a) <= tolerance
        if best.sum() < 20:
            return None
    selected = points[best]
    lo, hi = np.quantile(selected[:, 0], [0.02, 0.98])
    if hi - lo < min_length or abs(a) > np.tan(np.deg2rad(20)):
        return None
    occupied = np.unique(np.floor(selected[:, 0] / 0.1))
    coverage = len(occupied) / (occupied[-1] - occupied[0] + 1)
    if coverage < 0.6 or np.diff(occupied).max(initial=0) > 5:
        return None
    residuals = (selected[:, 1] - a * selected[:, 0] - b) / np.hypot(1, a)
    return dict(a=float(a), b=float(b), lo=float(lo), hi=float(hi),
                inliers=selected, rms=float(np.sqrt(np.mean(residuals ** 2))),
                coverage=float(coverage))


def detect_corridor(frame, robot_pose=None, max_width=1.5, obstacle_class=2):
    if not np.isfinite(max_width) or max_width <= 0:
        raise ValueError("窄通道宽度阈值必须为正。")
    pose = estimate_robot_pose(frame) if robot_pose is None else np.asarray(robot_pose, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("无效的车辆位姿。")
    forward = np.array([np.cos(pose[2]), np.sin(pose[2])])
    left = np.array([-forward[1], forward[0]])
    basis = np.array([forward, left])
    points = obstacle_points(frame["local_map"], obstacle_class)
    local = (points - pose[:2]) @ basis.T
    fits = {"left": fit_side(local[local[:, 1] > 0.15]),
            "right": fit_side(local[local[:, 1] < -0.15])}
    result = dict(status="insufficient_evidence", coordinate_system="world", unit="m",
                  obstacle_class=obstacle_class, robot_pose=pose.tolist(),
                  narrow_width_threshold_m=max_width, is_narrow=None,
                  method="heading-constrained RANSAC + total least squares",
                  width_definition="cross section perpendicular to vehicle heading",
                  boundaries={})

    def to_world(s, lateral):
        return np.column_stack([s, lateral]) @ basis + pose[:2]

    for side, fit in fits.items():
        if fit is None:
            continue
        s = np.linspace(fit["lo"], fit["hi"], int(np.ceil((fit["hi"] - fit["lo"]) / 0.05)) + 1)
        samples = to_world(s, fit["a"] * s + fit["b"])
        normal = (left - fit["a"] * forward) / np.hypot(1, fit["a"])
        c = -normal @ pose[:2] - fit["b"] / np.hypot(1, fit["a"])
        result["boundaries"][side] = dict(
            endpoints=samples[[0, -1]].tolist(), samples=samples.tolist(),
            line_abc=[*normal.tolist(), float(c)],
            inlier_points=(fit["inliers"] @ basis + pose[:2]).tolist(),
            inlier_count=len(fit["inliers"]), rms_error_m=fit["rms"],
            longitudinal_coverage=fit["coverage"],
        )
    if any(fit is None for fit in fits.values()):
        return result
    lf, rf = fits["left"], fits["right"]
    lo, hi = max(lf["lo"], rf["lo"]), min(lf["hi"], rf["hi"])
    angle = abs(np.arctan(lf["a"]) - np.arctan(rf["a"]))
    if hi - lo < 1.0 or not lo <= 0 <= hi or angle > np.deg2rad(10):
        result["status"] = "no_consistent_corridor"
        return result
    s = np.linspace(lo, hi, int(np.ceil((hi - lo) / 0.05)) + 1)
    left_lateral, right_lateral = lf["a"] * s + lf["b"], rf["a"] * s + rf["b"]
    widths = left_lateral - right_lateral
    if np.any(left_lateral <= 0) or np.any(right_lateral >= 0):
        result["status"] = "no_consistent_corridor"
        return result
    result.update(status="ok", is_narrow=bool(widths.max() <= max_width),
                  width_at_robot_m=lf["b"] - rf["b"],
                  width_min_m=float(widths.min()), width_max_m=float(widths.max()),
                  observed_overlap_m=float(hi - lo),
                  cross_sections=[dict(left=l.tolist(), right=r.tolist(), width_m=float(w))
                                  for l, r, w in zip(to_world(s, left_lateral),
                                                     to_world(s, right_lateral), widths)])
    return result


def draw_corridor(ax, result):
    handles = []
    for side, color in [("left", "#a855f7"), ("right", "#f97316")]:
        boundary = result["boundaries"].get(side)
        if boundary is None:
            continue
        points = np.asarray(boundary["samples"])
        line, = ax.plot(points[:, 0], points[:, 1], color=color, lw=2.5,
                        zorder=7, label=f"Fitted {side} boundary")
        handles.append(line)
        ends = np.asarray(boundary["endpoints"])
        ax.scatter(ends[:, 0], ends[:, 1], color=color, s=20, zorder=8)
    if result["status"] == "ok":
        sections = result["cross_sections"]
        for section in sections[::max(1, len(sections) // 8)]:
            points = np.array([section["left"], section["right"]])
            ax.plot(points[:, 0], points[:, 1], color="#64748b", ls="--", lw=0.6, zorder=2)
        text = f"Fitted corridor width: {result['width_at_robot_m']:.3f} m"
    else:
        text = f"Corridor: {result['status']}"
    ax.text(0.02, 0.93, text, transform=ax.transAxes, va="top", fontsize=9)
    return handles


def main():
    parser = argparse.ArgumentParser(description="从原始 gridmap 拟合窄通道左右边界，输出 world 坐标。")
    parser.add_argument("input", nargs="?", type=Path, default=Path("frame.json"))
    parser.add_argument("-o", "--output", type=Path, default=Path("corridor_boundaries.json"))
    parser.add_argument("--max-width", type=float, default=1.5, help="窄通道宽度阈值（米）")
    parser.add_argument("--obstacle-class", type=int, choices=[1, 2, 3], default=2)
    parser.add_argument("--robot-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error("输出不能覆盖输入 JSON。")
    frame = json.loads(args.input.read_text(encoding="utf-8"))
    result = detect_corridor(frame, args.robot_pose, args.max_width, args.obstacle_class)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k not in ("boundaries", "cross_sections")},
                     ensure_ascii=False, indent=2))
    for side, boundary in result["boundaries"].items():
        print(f"{side}: {boundary['endpoints']}; RMS={boundary['rms_error_m']:.4f} m")
    if args.show:
        import matplotlib.pyplot as plt
        from visualize_frame import plot_frame
        plot_frame(frame, args.input.name, corridor_result=result)
        plt.show()


if __name__ == "__main__":
    main()
