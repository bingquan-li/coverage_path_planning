"""Export the selected Rerun snapshot directly to JSON, without reading a .pdlog.

Run with the project's .venv/bin/python. The active recording must be queryable
through Rerun's catalog (open an .rrd, rather than a live SDK proxy stream).
"""

from __future__ import annotations

import argparse
import json
import queue
import subprocess
import sys
import tempfile
import threading
from pathlib import Path


def viewer_state(endpoint: str, rerun: str) -> dict:
    """Read the cursor without changing the viewer or its time selection."""
    with tempfile.TemporaryFile(mode="w+") as errors:
        process = subprocess.Popen(
            [rerun, "viewer-mcp", "--endpoint", endpoint],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=errors,
            text=True,
        )
        replies: queue.Queue[str | None] = queue.Queue()

        def read_replies() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                replies.put(line)
            replies.put(None)

        threading.Thread(target=read_replies, daemon=True).start()

        def send(message: dict) -> None:
            assert process.stdin is not None
            process.stdin.write(json.dumps(message) + "\n")
            process.stdin.flush()

        def request(request_id: int, method: str, params: dict) -> dict:
            send(
                {"jsonrpc": "2.0", "id": request_id, "method": method, "params": params}
            )
            while True:
                try:
                    line = replies.get(timeout=30)
                except queue.Empty as error:
                    raise RuntimeError(
                        "Rerun Viewer did not respond within 30 seconds"
                    ) from error
                if line is None:
                    errors.seek(0)
                    raise RuntimeError(
                        "Cannot read Rerun Viewer: " + errors.read().strip()
                    )
                response = json.loads(line)
                if response.get("id") != request_id:
                    continue
                if "error" in response:
                    raise RuntimeError(str(response["error"]))
                return response["result"]

        try:
            request(
                1,
                "initialize",
                {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "pdlog-frame-export", "version": "1"},
                },
            )
            send({"jsonrpc": "2.0", "method": "notifications/initialized"})
            result = request(
                2,
                "tools/call",
                {
                    "name": "rerun_get_viewer_state",
                    "arguments": {},
                },
            )
            if result.get("isError"):
                raise RuntimeError(str(result.get("content")))
            if "structuredContent" in result:
                return result["structuredContent"]
            for content in result.get("content", []):
                if content.get("type") == "text":
                    return json.loads(content["text"])
            raise RuntimeError("Rerun returned no viewer state")
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def cursor_selection(state: dict) -> tuple[str, int, str]:
    active = state.get("active_store_id")
    for recording in state.get("recordings", []):
        if recording.get("store_id") != active:
            continue
        cursor = recording.get("current_time") or {}
        timeline = cursor.get("timeline", {}).get("name")
        if timeline not in ("robot_time", "log_line"):
            raise ValueError(
                "请在 Rerun 中选择 robot_time 或 log_line 时间线，再定位到要导出的时刻。"
            )
        if "time" not in cursor:
            raise ValueError("当前记录没有选定时间。")
        # protobuf 的零值可能省略 time 字段。
        return timeline, int(cursor["time"].get("time", 0)), active  # type: ignore
    raise ValueError("Rerun 当前没有打开的记录。")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="新的 JSON 文件，不覆盖现有文件")
    parser.add_argument("--viewer", default="http://127.0.0.1:9876")
    parser.add_argument("--rerun", default="rerun", help="支持 viewer-mcp 的 Rerun CLI")
    parser.add_argument("--timeline", choices=["robot_time", "log_line"])
    parser.add_argument("--at", type=int, help="可选：robot_time 纳秒数或 log_line 行号")
    args = parser.parse_args()
    if (args.timeline is None) != (args.at is None):
        parser.error("--timeline 和 --at 必须一起提供")
    try:
        from rerun_frame_data import export_viewer_snapshot

        if args.output.exists() or args.output.is_symlink():
            raise ValueError(f"输出文件已存在，未覆盖：{args.output}")
        state = viewer_state(args.viewer, args.rerun)
        if args.timeline is None:
            timeline, time, store_id = cursor_selection(state)
        else:
            timeline, time, store_id = args.timeline, args.at, state.get("active_store_id")
            if not store_id:
                raise ValueError("Rerun 当前没有打开的记录。")
        snapshot = export_viewer_snapshot(state, timeline, time, store_id)
        # 先完成编码检查，再排他创建文件，避免失败时留下错误的 JSON。
        encoded = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        with args.output.open("x", encoding="utf-8") as output:
            output.write(encoded + "\n")
        print(f"已从 Rerun 导出 {timeline}={time}：{args.output}")
        return 0
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f"export_current_frame: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
