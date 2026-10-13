"""A* over frame.json using NumPy and SciPy (available in the project .venv).

Class 2 is ignored; class 1 becomes a lethal obstacle (255). Class 3 is
conservatively blocked. Inflation is a traversable cost, not a hard obstacle.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class SearchMap:
    costs: np.ndarray
    resolution: float
    origin: np.ndarray
    rotation: np.ndarray
    known: np.ndarray

    def world_to_cell(self, point):
        xy = (np.asarray(point) - self.origin) @ self.rotation / self.resolution
        col, row = np.floor(xy).astype(int)
        return int(row), int(col)

    def cell_to_world(self, cell):
        row, col = cell
        return self.origin + self.rotation @ (
            np.array([col + 0.5, row + 0.5]) * self.resolution
        )


def inflate_costs(cells, resolution: float, radius: float = 1.0):
    """Euclidean, linear falloff: 255 - 195*d/r for 0 < d <= r.

    Overlapping disks take the maximum cost, equivalently the nearest obstacle.
    Float costs avoid rounding cells adjacent to an obstacle up to lethal 255.
    """
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("地图分辨率必须为正。")
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("膨胀半径必须为正。")
    cells = np.asarray(cells)
    if cells.ndim != 2 or not np.isin(cells, [0, 1, 2, 3]).all():
        raise ValueError("地图必须是类别为 0～3 的二维数组。")
    obstacles = cells == 2
    costs = np.zeros(cells.shape, dtype=float)
    if obstacles.any():
        distances = distance_transform_edt(~obstacles, sampling=resolution)
        inside = distances <= radius
        costs[inside] = 255.0 - 195.0 * distances[inside] / radius
    costs[cells == 3] = 255.0
    return costs


def astar(costs, start, goal, resolution: float, cost_weight: float = 5.0):
    """Eight-connected A*, with no diagonal corner cutting.

    Edge cost = length in metres * (1 + weight * mean endpoint cost / 255).
    Euclidean distance is an admissible heuristic. Returns cells, or [] if blocked.
    """
    if not math.isfinite(cost_weight) or cost_weight < 0:
        raise ValueError("代价权重必须是非负有限数值。")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("地图分辨率必须为正。")
    costs = np.asarray(costs, dtype=float)
    if costs.ndim != 2 or not np.isfinite(costs).all() or (costs < 0).any():
        raise ValueError("无效的代价地图。")
    height, width = costs.shape

    def valid(cell):
        r, c = cell
        return 0 <= r < height and 0 <= c < width and costs[r, c] < 255

    if not valid(start) or not valid(goal):
        return []
    distances = np.full(costs.shape, np.inf)
    parents = {}
    distances[start] = 0.0
    queue = [(math.dist(start, goal) * resolution, 0.0, start)]
    moves = [
        (dr, dc, math.hypot(dr, dc) * resolution)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if dr or dc
    ]
    while queue:
        _, distance, current = heapq.heappop(queue)
        if distance > distances[current]:
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = parents[current]
                path.append(current)
            return path[::-1]
        r, c = current
        for dr, dc, length in moves:
            neighbor = (r + dr, c + dc)
            if not valid(neighbor):
                continue
            if dr and dc and (not valid((r + dr, c)) or not valid((r, c + dc))):
                continue
            candidate = distance + length * (
                1 + cost_weight * (costs[current] + costs[neighbor]) / (2 * 255)
            )
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                parents[neighbor] = current
                heuristic = math.dist(neighbor, goal) * resolution
                heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
    return []


def estimate_robot_pose(frame):
    """User-selected convention: map center and ordered footprint front/rear."""
    position = np.asarray(frame["local_map"]["pos"], dtype=float)
    footprint = np.asarray((frame.get("foot_print") or {}).get("data", []), dtype=float)
    if (
        position.shape != (2,)
        or not np.isfinite(position).all()
        or footprint.ndim != 2
        or footprint.shape[1] != 2
        or len(footprint) < 7
        or not np.isfinite(footprint).all()
    ):
        raise ValueError(
            "无法从地图中心和轮廓估算位姿，请通过 --robot-pose X Y YAW 指定。"
        )
    front = (footprint[1] + footprint[2]) / 2
    rear = (footprint[5] + footprint[6]) / 2
    forward = front - rear
    if np.linalg.norm(forward) < 1e-8:
        raise ValueError("轮廓前后端重合，无法估算航向。")
    return np.array([*position, math.atan2(forward[1], forward[0])])


def build_search_map(grid, start, goal):
    cells = np.asarray(grid["data"])
    if cells.shape != (grid["height"], grid["width"]):
        raise ValueError("地图尺寸不匹配。")
    row_order = grid.get("row_order", "bottom_to_top")
    if row_order == "top_to_bottom":
        cells = cells[::-1]
    elif row_order != "bottom_to_top":
        raise ValueError("未知的地图行序。")
    resolution = float(grid["map_scale"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("地图分辨率必须为正。")
    origin = np.asarray(grid["origin"], dtype=float)
    q = np.asarray(grid.get("rotation_xyzw", [0, 0, 0, 1]), dtype=float)
    if (
        origin.shape != (2,)
        or not np.isfinite(origin).all()
        or q.shape != (4,)
        or not np.isfinite(q).all()
        or np.linalg.norm(q) == 0
    ):
        raise ValueError("无效的地图位姿。")
    q /= np.linalg.norm(q)
    if not np.allclose(q[:2], 0):
        raise ValueError("只支持绕 Z 轴旋转的二维地图。")
    yaw = 2 * math.atan2(q[2], q[3])
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    height, width = cells.shape
    # Padding preserves the original grid lattice. Unknown exterior is free by
    # explicit user choice; inflate AFTER padding so obstacles affect it too.
    endpoint_xy = (np.asarray([start, goal]) - origin) @ rotation / resolution
    center = np.array([width, height]) / 2
    lower = np.floor(
        np.minimum.reduce(
            [
                np.zeros(2),
                center - 3 / resolution,
                endpoint_xy.min(axis=0) - 1 / resolution,
            ]
        )
    ).astype(int)
    upper = np.ceil(
        np.maximum.reduce(
            [
                np.array([width, height]),
                center + 3 / resolution,
                endpoint_xy.max(axis=0) + 1 / resolution,
            ]
        )
    ).astype(int)
    left, bottom = -lower
    expanded = np.zeros((upper[1] - lower[1], upper[0] - lower[0]), dtype=cells.dtype)
    known = np.zeros(expanded.shape, dtype=bool)
    expanded[bottom : bottom + height, left : left + width] = cells
    known[bottom : bottom + height, left : left + width] = True
    return SearchMap(
        inflate_costs(expanded, resolution),
        resolution,
        origin + rotation @ (lower * resolution),
        rotation,
        known,
    )


def plan_frame(frame, robot_pose=None, cost_weight: float = 5.0):
    pose = (
        estimate_robot_pose(frame)
        if robot_pose is None
        else np.asarray(robot_pose, dtype=float)
    )
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("车辆位姿必须为有限的 X、Y、YAW（弧度）。")
    start = pose[:2]
    goal = start - 3.0 * np.array([math.cos(pose[2]), math.sin(pose[2])])
    search_map = build_search_map(frame["local_map"], start, goal)
    start_cell, goal_cell = map(search_map.world_to_cell, [start, goal])
    cells = astar(
        search_map.costs, start_cell, goal_cell, search_map.resolution, cost_weight
    )
    points = np.array([search_map.cell_to_world(cell) for cell in cells]).reshape(-1, 2)
    if len(points):
        # Preserve the exact requested endpoints (within their traversable cells).
        points = np.vstack([start, points, goal])
    status = "ok" if cells else "no_path"
    if search_map.costs[start_cell] >= 255:
        status = "start_blocked"
    elif search_map.costs[goal_cell] >= 255:
        status = "goal_blocked"
    result = {
        "status": status,
        "coordinate_system": "world",
        "unit": "m",
        "robot_pose": pose.tolist(),
        "pose_source": "map_center_and_footprint" if robot_pose is None else "explicit",
        "start": start.tolist(),
        "goal": goal.tolist(),
        "rear_distance_m": 3.0,
        "inflation_radius_m": 1.0,
        "inflation_edge_cost": 60,
        "cost_weight": cost_weight,
        "outside_map": "assumed_free",
        "class_mapping": {"0": 0, "1": 0, "2": 255, "3": 255},
        "path": points.tolist(),
        "length_m": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        "outside_map_path_cells": sum(
            not bool(search_map.known[cell]) for cell in cells
        ),
    }
    return result, search_map


def main():
    parser = argparse.ArgumentParser(
        description="过滤类别 2、膨胀类别 1，并 A* 搜索到车辆后方 3 米。"
    )
    parser.add_argument("input", nargs="?", type=Path, default=Path("frame.json"))
    parser.add_argument("-o", "--output", type=Path, default=Path("astar_path.json"))
    parser.add_argument("--robot-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--cost-weight", type=float, default=5.0)
    parser.add_argument(
        "--show", action="store_true", help="显示地图及搜索路径"
    )
    parser.add_argument(
        "--show-inflation", action="store_true", help="绘图时显示膨胀代价区域和色条，默认隐藏"
    )
    args = parser.parse_args()
    if args.output.resolve() == args.input.resolve():
        parser.error("输出不能覆盖输入 JSON。")
    with args.input.open(encoding="utf-8") as source:
        frame = json.load(source)
    result, search_map = plan_frame(frame, args.robot_pose, args.cost_weight)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        f"A*: {result['status']}; length={result['length_m']:.3f} m; "
        f"goal={result['goal']}; saved={args.output}"
    )
    if args.show:
        import matplotlib.pyplot as plt
        from visualize_frame import plot_frame

        plot_frame(frame, args.input.name, astar_result=(result, search_map),
                   show_inflation=args.show_inflation)
        plt.show()
    if result["status"] != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
