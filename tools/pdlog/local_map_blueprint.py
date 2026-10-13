"""重新生成 C++ 示例使用的 Blueprint：.venv/bin/python tools/pdlog/local_map_blueprint.py。"""

from pathlib import Path

import rerun.blueprint as rrb

if __name__ == "__main__":
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="Local Map",
                origin="/world",
                contents=["/world/**"],
            ),
            rrb.Vertical(
                rrb.Tabs(
                    rrb.TextDocumentView(
                        name="Robot State",
                        origin="/navigation/current",
                    ),
                    rrb.TextLogView(
                        name="State History",
                        origin="/navigation/events",
                        columns=rrb.TextLogColumns(text_log_columns=["Body"]),
                    ),
                    active_tab=0,
                ),
                rrb.TimeSeriesView(
                    name="Linear Speed (m/s)",
                    origin="/navigation/linear_speed",
                    axis_x=rrb.TimeAxis(link=rrb.components.LinkAxis.LinkToGlobal),
                ),
                rrb.TimeSeriesView(
                    name="Angular Speed (rad/s)",
                    origin="/navigation/angular_speed",
                    axis_x=rrb.TimeAxis(link=rrb.components.LinkAxis.LinkToGlobal),
                ),
                row_shares=[0.34, 0.33, 0.33],
            ),
            column_shares=[0.65, 0.35],
        ),
        rrb.TimePanel(timeline="robot_time", state="expanded"),
        rrb.SelectionPanel(state="collapsed"),
        auto_layout=False,
        auto_views=False,
    )
    # 必须与 C++ RecordingStream 的 application_id 一致。
    blueprint.save("local_map", Path(__file__).with_suffix(".rbl"))
