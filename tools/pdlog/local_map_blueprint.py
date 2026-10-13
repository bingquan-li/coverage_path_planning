"""重新生成 C++ 示例使用的 Blueprint：.venv/bin/python tools/pdlog/local_map_blueprint.py。"""

from pathlib import Path

import rerun.blueprint as rrb

if __name__ == "__main__":
    blueprint = rrb.Blueprint(
        rrb.Spatial3DView(
            name="Local Map",
            origin="/world",
            contents=["/world/**"],
        ),
        rrb.TimePanel(timeline="frame", state="expanded"),
        auto_layout=False,
        auto_views=False,
    )
    # 必须与 C++ RecordingStream 的 application_id 一致。
    blueprint.save("local_map", Path(__file__).with_suffix(".rbl"))
