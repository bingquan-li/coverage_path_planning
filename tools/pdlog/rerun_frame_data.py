"""Query a single snapshot from a Rerun catalog; never read a .pdlog."""

from __future__ import annotations

import math
import re
from urllib.parse import parse_qs, urlsplit

MAP = "/world/local_map:GridMap:"
FOOTPRINT = "/world/robot/footprint:LineStrips3D:strips"
GLOBAL_PATHS = "/world/global_paths:LineStrips3D:strips"
LINEAR = "/navigation/linear_speed:Scalars:scalars"
ANGULAR = "/navigation/angular_speed:Scalars:scalars"
STATE = "/navigation/current:TextDocument:text"
COLUMNS = [
    MAP + field for field in ("data", "format", "cell_size", "translation", "quaternion")
] + [FOOTPRINT, GLOBAL_PATHS, LINEAR, ANGULAR, STATE]


def catalog_target(state: dict, store_id: str) -> tuple[str, str, str]:
    """Use the active recording's exact dataset and segment, never its display name."""
    url = urlsplit(state.get("url", ""))
    path = url.path.strip("/").split("/")
    if url.scheme not in ("rerun+http", "rerun+https") or len(path) != 2 or path[0] != "dataset":
        raise ValueError(
            "当前记录是实时流，未提供 catalog 数值查询。请先在 Rerun 中 Ctrl+S 保存为 .rrd，"
            "再打开该 .rrd 并选中目标时刻，然后重新导出。无需原始 .pdlog。"
        )
    segment = store_id.rsplit(":", 1)[-1]
    url_segments = parse_qs(url.query).get("segment_id", [])
    if url_segments != [segment]:
        raise ValueError("当前 Viewer URL 与激活记录不一致，请选中目标记录后重试。")
    return f"{url.scheme}://{url.netloc}", path[1], segment


def query_row(dataset, segment: str, timeline: str, time: int) -> dict:
    import pyarrow as pa

    if timeline not in ("robot_time", "log_line"):
        raise ValueError("请选择 robot_time 或 log_line 时间线。")
    index_type = pa.duration("ns") if timeline == "robot_time" else pa.int64()
    view = dataset.filter_segments(segment).filter_contents([
        "/world/local_map", "/world/robot/footprint", "/world/global_paths", "/navigation/linear_speed",
        "/navigation/angular_speed", "/navigation/current",
    ])
    # 一个 segment、一个时间点、仅需要的列。服务端做 latest-at，不下载整个记录。
    reader = view.reader(
        index=timeline,
        using_index_values={segment: pa.array([time], type=index_type)},
        fill_latest_at=True,
        include_semantically_empty_columns=True,
        include_tombstone_columns=True,
    )
    columns = [name for name in COLUMNS if name in reader.schema().names]
    if MAP + "data" not in columns:
        raise ValueError("当前记录没有 /world/local_map 的 GridMap 数据。")
    rows = [row for batch in reader.select(*columns).collect() for row in batch.to_pylist()]
    if len(rows) != 1:
        raise ValueError("没有找到选中记录的唯一快照，请确认记录已加载完成。")
    return rows[0]


def _single(row: dict, column: str, *, required: bool = False):
    values = row.get(column)
    if values is None or len(values) == 0:
        if required:
            raise ValueError(f"选中时刻没有有效数据：{column}")
        return None
    if len(values) != 1:
        raise ValueError(f"预期单个数据实例：{column}")
    if required and values[0] is None:
        raise ValueError(f"选中时刻没有有效数据：{column}")
    return values[0]


def decode_snapshot(row: dict, timeline: str, time: int, store_id: str) -> dict:
    pixels = _single(row, MAP + "data", required=True)
    image_format = _single(row, MAP + "format", required=True)
    # Rerun ImageFormat: ColorModel::L=1, ChannelDatatype::U8=6。
    if (image_format.get("color_model"), image_format.get("channel_datatype")) != (1, 6) or image_format.get("pixel_format") is not None:
        raise ValueError("只支持本工具写入的 L/U8 GridMap，不能把其他图像格式解释为地图类别。")
    width, height = image_format["width"], image_format["height"]
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError("GridMap 缓冲区长度与宽高不一致。")
    classes = {0: 0, 32: 1, 192: 2, 254: 3}
    try:
        # 恢复地图的底部行在前及 0～3 类别，仅处理当前帧的像素。
        cells = [
            [classes[value] for value in pixels[y * width:(y + 1) * width]]
            for y in range(height - 1, -1, -1)
        ]
    except KeyError as error:
        raise ValueError(f"遇到未知色图索引 {error.args[0]}，不能无损还原地图类别。") from error
    resolution = _single(row, MAP + "cell_size", required=True)
    translation = _single(row, MAP + "translation", required=True)
    quaternion = _single(row, MAP + "quaternion") or [0.0, 0.0, 0.0, 1.0]
    if not math.isfinite(resolution) or resolution <= 0 or len(translation) != 3 or len(quaternion) != 4:
        raise ValueError("无效的 GridMap 分辨率或位姿。")
    if not all(math.isfinite(v) for v in [*translation, *quaternion]):
        raise ValueError("GridMap 位姿包含非有限数值。")
    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm == 0:
        raise ValueError("GridMap 四元数不能为零。")
    x, y, z, w = [v / norm for v in quaternion]
    hx, hy = width * resolution / 2, height * resolution / 2
    center = [
        translation[0] + (1 - 2 * (y * y + z * z)) * hx + 2 * (x * y - z * w) * hy,
        translation[1] + 2 * (x * y + z * w) * hx + (1 - 2 * (x * x + z * z)) * hy,
    ]
    local_map = {
        "type": "local_map", "width": width, "height": height,
        "map_size": width if width == height else None, "map_scale": resolution,
        "pos": center, "origin": translation[:2], "translation": translation,
        "rotation_xyzw": quaternion, "data": cells, "encoding": "class_id_0_to_3",
        "row_order": "bottom_to_top", "coordinate_system": "world",
    }
    strip = _single(row, FOOTPRINT)
    footprint = None
    if strip:
        if any(len(point) != 3 for point in strip):
            raise ValueError("无效的机器人轮廓顶点。")
        footprint = {
            "type": "foot_print", "data": [point[:2] for point in strip],
            "data_xyz": strip, "coordinate_system": "world", "unit": "m",
        }
    linear, angular, text = (_single(row, field) for field in (LINEAR, ANGULAR, STATE))
    status = None
    if text:
        match = re.search(r"^Status:\s*(\S+)", text, re.MULTILINE)
        if not match:
            raise ValueError("当前状态文本格式不受支持，无法提取状态名称。")
        status = match.group(1)
    navigation = None if all(v is None for v in (linear, angular, status)) else {
        "linear_speed": linear, "linear_speed_unit": "m/s",
        "angular_speed": angular, "angular_speed_unit": "rad/s", "status": status,
    }
    # 每个 LineStrip 对应一个原始分段，保留编号、重复点及空段。
    strips = row.get(GLOBAL_PATHS)
    global_paths = None
    if strips:
        if any(len(point) != 3 for strip in strips for point in strip):
            raise ValueError("无效的全局路径顶点。")
        global_paths = {
            "type": "global_paths", "coordinate_system": "world", "unit": "m",
            "total_number_of_segments": len(strips),
            "segments": [
                {"number_of_segment": i, "data": [point[:2] for point in strip], "data_xyz": strip}
                for i, strip in enumerate(strips)
            ],
        }
    return {
        "schema_version": 2, "data_source": "rerun",
        "viewer_recording": store_id, "selection": {"timeline": timeline, "time": time},
        "local_map": local_map, "foot_print": footprint, "navigation": navigation,
        "global_paths": global_paths,
    }


def export_viewer_snapshot(state: dict, timeline: str, time: int, store_id: str) -> dict:
    address, dataset_id, segment = catalog_target(state, store_id)
    try:
        import rerun as rr
    except ImportError as error:
        raise RuntimeError("请使用项目 Python 环境：.venv/bin/python tools/pdlog/export_current_frame.py …") from error
    client = rr.catalog.CatalogClient(address)
    dataset = client.get_dataset(id=dataset_id)
    return decode_snapshot(query_row(dataset, segment, timeline, time), timeline, time, store_id)
