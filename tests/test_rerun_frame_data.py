from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/pdlog"))
from rerun_frame_data import (  # noqa: E402
    ANGULAR, FOOTPRINT, GLOBAL_PATHS, LINEAR, MAP, STATE, catalog_target, decode_snapshot,
)


def sample():
    return {
        MAP + "data": [[192, 254, 0, 32]],
        MAP + "format": [{"width": 2, "height": 2, "color_model": 1,
                          "channel_datatype": 6, "pixel_format": None}],
        MAP + "cell_size": [0.5],
        MAP + "translation": [[-1.5, 1.5, 0.0]],
        MAP + "quaternion": [[0.0, 0.0, 0.0, 1.0]],
        FOOTPRINT: [[[-3.0, -2.0, 0.02], [-2.0, -1.0, 0.02], [-3.0, -1.0, 0.02]]],
        LINEAR: [-0.067762612345], ANGULAR: [-0.093752712345],
        STATE: ["Status: Navigating\n\nLinear speed: -0.0677626 m/s\n"],
    }


class RerunFrameDataTest(unittest.TestCase):
    def decode(self, row):
        return decode_snapshot(row, "robot_time", 123456789, "Recording:dataset:rec_1")

    def test_values_orientation_and_precision(self):
        result = self.decode(sample())
        self.assertEqual(result["local_map"]["data"], [[0, 1], [2, 3]])
        self.assertEqual(result["local_map"]["pos"], [-1.0, 2.0])
        self.assertEqual(result["foot_print"]["data"][0], [-3.0, -2.0])
        self.assertEqual(result["foot_print"]["data_xyz"][0][2], 0.02)
        self.assertEqual(result["navigation"]["linear_speed"], -0.067762612345)
        self.assertEqual(result["navigation"]["status"], "Navigating")
        self.assertEqual(result["data_source"], "rerun")
        self.assertNotIn("source_file", result)

    def test_missing_or_cleared_components(self):
        for empty in (None, []):
            row = sample()
            for key in (FOOTPRINT, LINEAR, ANGULAR, STATE):
                row[key] = empty
            result = self.decode(row)
            self.assertIsNone(result["foot_print"])
            self.assertIsNone(result["navigation"])

    def test_unknown_state_and_partial_navigation(self):
        row = sample()
        row[STATE] = ["Status: NewState"]
        row[LINEAR] = None
        result = self.decode(row)
        self.assertEqual(result["navigation"]["status"], "NewState")
        self.assertIsNone(result["navigation"]["linear_speed"])

    def test_bad_formats_do_not_silently_corrupt(self):
        changes = [
            (MAP + "data", None), (MAP + "data", [[0, 32]]),
            (MAP + "data", [[192, 255, 0, 32]]),
            (MAP + "cell_size", [-1]), (MAP + "cell_size", [float("nan")]),
            (MAP + "translation", [[float("inf"), 0, 0]]),
            (MAP + "quaternion", [[0, 0, 0, 0]]),
            (LINEAR, [1, 2]), (STATE, ["unrecognized text"]),
        ]
        for column, value in changes:
            with self.subTest(column=column, value=value):
                row = sample()
                row[column] = value
                with self.assertRaises(ValueError):
                    self.decode(row)
        row = sample()
        row[MAP + "format"][0]["color_model"] = 3
        with self.assertRaises(ValueError):
            self.decode(row)

    def test_recording_identity_and_live_proxy(self):
        state = {"url": "rerun+http://127.0.0.1:19876/dataset/ds?segment_id=rec_1"}
        self.assertEqual(catalog_target(state, "Recording:ds:rec_1"),
                         ("rerun+http://127.0.0.1:19876", "ds", "rec_1"))
        with self.assertRaises(ValueError):
            catalog_target(state, "Recording:ds:another")
        with self.assertRaisesRegex(ValueError, "Ctrl\\+S"):
            catalog_target({"url": "rerun+http://127.0.0.1:9876/proxy"}, "Recording:local_map:rec_1")

    def test_rotated_center(self):
        row = sample()
        row[MAP + "quaternion"] = [[0, 0, 1, 0]]
        result = self.decode(row)
        self.assertEqual(result["local_map"]["pos"], [-2.0, 1.0])

    def test_global_paths_segments_clear_and_old_recordings(self):
        row = sample()
        self.assertIsNone(self.decode(row)["global_paths"])
        row[GLOBAL_PATHS] = [
            [[-2.66, -1.47, 0.04], [-2.64, -1.42, 0.04], [-2.66, -1.47, 0.04]],
            [], [[10.0, 20.0, 0.04]],
        ]
        paths = self.decode(row)["global_paths"]
        self.assertEqual(paths["total_number_of_segments"], 3)
        self.assertEqual(paths["segments"][0]["data"], [[-2.66, -1.47], [-2.64, -1.42], [-2.66, -1.47]])
        self.assertEqual(paths["segments"][1]["data"], [])
        self.assertEqual(paths["segments"][2]["number_of_segment"], 2)
        row[GLOBAL_PATHS] = []
        self.assertIsNone(self.decode(row)["global_paths"])
        row[GLOBAL_PATHS] = [[[1, 2]]]
        with self.assertRaises(ValueError):
            self.decode(row)


if __name__ == "__main__":
    unittest.main()
