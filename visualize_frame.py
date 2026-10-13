"""Plot a frame's grid map, global paths, inflated costs and A* path.

Usage: .venv/bin/python visualize_frame.py frame.json --show
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def plot_frame(frame: dict, title: str, view_size: float = 6.0, astar_result=None,
               show_inflation: bool = False, corridor_result=None):
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch
    from matplotlib.transforms import Affine2D

    if not np.isfinite(view_size) or view_size <= 0:
        raise ValueError("显示范围必须是正的有限数值。")

    grid = frame["local_map"]
    cells = np.asarray(grid["data"])
    width, height = grid["width"], grid["height"]
    if cells.shape != (height, width) or not np.isin(cells, [0, 1, 2, 3]).all():
        raise ValueError("地图尺寸不匹配，或存在 0～3 以外的类别。")
    resolution = float(grid["map_scale"])
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("map_scale 必须是正的有限数值。")
    row_order = grid.get("row_order", "bottom_to_top")
    if row_order not in ("bottom_to_top", "top_to_bottom"):
        raise ValueError(f"不支持的行序：{row_order}")

    # The origin is the lower-left cell edge; pos is the map center.
    origin = np.asarray(grid["origin"], dtype=float)
    quaternion = np.asarray(grid.get("rotation_xyzw", [0, 0, 0, 1]), dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("origin 必须是有效的二维坐标。")
    if (quaternion.shape != (4,) or not np.isfinite(quaternion).all()
            or np.linalg.norm(quaternion) == 0):
        raise ValueError("无效的地图旋转四元数。")
    qx, qy, qz, qw = quaternion / np.linalg.norm(quaternion)
    if not np.allclose([qx, qy], 0):
        raise ValueError("此二维可视化仅支持绕 Z 轴旋转的地图。")
    yaw = 2 * np.arctan2(qz, qw)
    map_transform = Affine2D().rotate(yaw).translate(*origin)
    size_x, size_y = width * resolution, height * resolution

    colors = ["#f1f5f9", "#60a5fa", "#263445", "#f59e0b"]
    cmap = ListedColormap(colors)
    norm = BoundaryNorm(np.arange(-0.5, 4.5), cmap.N)
    fig, ax = plt.subplots(figsize=(10, 11), layout="constrained")
    ax.imshow(
        cells, cmap=cmap, norm=norm, interpolation="nearest",
        origin="lower" if row_order == "bottom_to_top" else "upper",
        extent=(0, size_x, 0, size_y),
        transform=map_transform + ax.transData,
    )
    corners = map_transform.transform(
        [[0, 0], [size_x, 0], [size_x, size_y], [0, size_y], [0, 0]]
    )
    ax.plot(corners[:, 0], corners[:, 1], color="#94a3b8", linewidth=0.8)

    classes, counts = np.unique(cells, return_counts=True)
    handles = [
        Patch(facecolor=colors[int(value)], edgecolor="#94a3b8",
              label=f"Class {int(value)} ({count:,} cells)")
        for value, count in zip(classes, counts)
    ]
    if astar_result is None:
        from astar_frame import plan_frame
        astar_result = plan_frame(frame)
    search, search_map = astar_result
    if show_inflation:
        cost_transform = Affine2D.from_values(
            search_map.rotation[0, 0], search_map.rotation[1, 0],
            search_map.rotation[0, 1], search_map.rotation[1, 1],
            *search_map.origin,
        )
        cost_height, cost_width = search_map.costs.shape
        cost_image = ax.imshow(
            np.ma.masked_where(search_map.costs == 0, search_map.costs),
            cmap="YlOrRd", vmin=60, vmax=255, alpha=0.65,
            origin="lower", interpolation="nearest", zorder=1,
            extent=(0, cost_width * resolution, 0, cost_height * resolution),
            transform=cost_transform + ax.transData,
        )
        fig.colorbar(cost_image, ax=ax, shrink=0.6, pad=0.02, label="A* cost (255 blocked)")
    if search["path"]:
        path = np.asarray(search["path"])
        line, = ax.plot(path[:, 0], path[:, 1], color="#2563eb", linewidth=2,
                        zorder=5, label=f"A* path ({search['length_m']:.2f} m)")
        handles.append(line)
    handles.append(ax.scatter(*search["start"], color="#2563eb", marker="o", s=35,
                              zorder=6, label="A* start"))
    handles.append(ax.scatter(*search["goal"], color="#2563eb", marker="*", s=110,
                              zorder=6, clip_on=False, label="Goal (3 m behind)"))
    global_paths = frame.get("global_paths")
    if global_paths:
        if global_paths.get("coordinate_system", "world") != "world":
            raise ValueError("全局路径必须使用 world 坐标。")
        path_handle = None
        for segment in global_paths.get("segments", []):
            if not segment.get("data"):
                continue
            points = np.asarray(segment["data"], dtype=float)
            if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
                raise ValueError("全局路径的 segment.data 必须是有限的二维坐标数组。")
            # Each segment is independent and already expressed in world coordinates.
            line, = ax.plot(points[:, 0], points[:, 1], color="#16a34a",
                            linewidth=1.8, marker="o" if len(points) == 1 else None,
                            markersize=4, zorder=3, label="Global path")
            if path_handle is None:
                path_handle = line
        if path_handle is not None:
            handles.append(path_handle)

    footprint = frame.get("foot_print")
    if footprint and footprint.get("data"):
        # Footprint vertices are already in world coordinates.
        vertices = np.asarray(footprint["data"], dtype=float)
        if vertices.ndim != 2 or vertices.shape[1] != 2 or not np.isfinite(vertices).all():
            raise ValueError("foot_print.data 必须是有限的二维坐标数组。")
        vertices = np.vstack([vertices, vertices[0]])
        line, = ax.plot(vertices[:, 0], vertices[:, 1], color="#e11d48",
                        linewidth=2, zorder=4, label="Robot footprint")
        handles.append(line)

    if corridor_result is not None:
        from detect_corridor import draw_corridor
        handles.extend(draw_corridor(ax, corridor_result))

    ax.set_aspect("equal")
    # Keep a fixed world-coordinate window centered on the local map.
    center = map_transform.transform([size_x / 2, size_y / 2])
    ax.set_xlim(center[0] - view_size / 2, center[0] + view_size / 2)
    ax.set_ylim(center[1] - view_size / 2, center[1] + view_size / 2)
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_title(f"{title} | Global path and A* ({search['status']})\n"
                 f"{width} x {height} cells | {resolution:.4g} m/cell | "
                 f"View: {view_size:g} x {view_size:g} m", pad=12)
    ignored = ", ".join(k for k, v in search["class_mapping"].items() if v == 0 and k != "0")
    ax.text(0.02, 0.98, f"A*: class {ignored} ignored; outside map assumed free",
            transform=ax.transAxes, va="top", fontsize=8, color="#475569")
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.09),
              ncol=min(len(handles), 3), frameon=False, fontsize=8)
    return fig


def main() -> None:
    parser = argparse.ArgumentParser(description="可视化 gridmap、全局路径、机器人轮廓和 A* 搜索结果。")
    parser.add_argument("input", nargs="?", type=Path, default=Path("frame.json"))
    parser.add_argument("-o", "--output", type=Path, help="输出图片，默认 <输入名>_gridmap.png")
    parser.add_argument("--show", action="store_true", help="保存后打开交互窗口，可缩放和平移")
    parser.add_argument("--view-size", type=float, default=6.0,
                        help="以地图中心为中心的正方形显示范围，单位米，默认 6")
    parser.add_argument("--robot-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--cost-weight", type=float, default=5.0)
    parser.add_argument("--show-inflation", action="store_true", help="显示膨胀代价区域和色条，默认隐藏")
    parser.add_argument("--show-boundaries", action="store_true", help="拟合并叠加通道左右边界")
    parser.add_argument("--save-only", action="store_true", help="只保存图片，不打开窗口")
    args = parser.parse_args()
    output = args.output or args.input.with_name(f"{args.input.stem}_gridmap.png")
    if output.resolve() == args.input.resolve():
        parser.error("输出路径不能覆盖输入 JSON。")

    import matplotlib
    if args.save_only:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from astar_frame import plan_frame

    with args.input.open(encoding="utf-8") as source:
        frame = json.load(source)
    result = plan_frame(frame, args.robot_pose, args.cost_weight)
    print(f"A*: {result[0]['status']}; length={result[0]['length_m']:.3f} m")
    corridor = None
    if args.show_boundaries:
        from detect_corridor import detect_corridor
        corridor = detect_corridor(frame, args.robot_pose)
    fig = plot_frame(frame, args.input.name, args.view_size, astar_result=result,
                     show_inflation=args.show_inflation, corridor_result=corridor)
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.save_only or args.output:
        fig.savefig(output, dpi=200, facecolor="white", bbox_inches="tight")
        print(f"Saved: {output.resolve()}")
    if not args.save_only:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
