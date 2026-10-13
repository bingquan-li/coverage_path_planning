import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detect_corridor import detect_corridor, fit_side, obstacle_points


def make_frame():
    cells = np.zeros((160, 160), dtype=int)
    cells[32:128, 56] = 2
    cells[32:128, 104] = 2
    cells[127, 30:130] = 2  # transverse wall must not become a side boundary
    cells[65:85, 110:125] = 1  # other classes do not influence structural fit
    return {"local_map": dict(data=cells.tolist(), width=160, height=160,
                              map_scale=0.025, origin=[-2, -2], pos=[0, 0],
                              row_order="bottom_to_top", rotation_xyzw=[0, 0, 0, 1])}


class CorridorTest(unittest.TestCase):
    def test_parallel_walls_and_output_coordinates(self):
        frame = make_frame()
        result = detect_corridor(frame, [0, 0, math.pi / 2])
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["is_narrow"])
        self.assertAlmostEqual(result["width_at_robot_m"], 1.2, delta=0.02)
        for boundary in result["boundaries"].values():
            points = np.asarray(boundary["samples"])
            a, b, c = boundary["line_abc"]
            np.testing.assert_allclose(points @ [a, b] + c, 0, atol=1e-10)
            self.assertGreater(boundary["longitudinal_coverage"], 0.9)
        self.assertFalse(detect_corridor(frame, [0, 0, math.pi / 2], max_width=1)["is_narrow"])

    def test_one_wall_and_empty_map_not_a_corridor(self):
        frame = make_frame()
        cells = np.zeros((160, 160), dtype=int)
        cells[32:128, 56] = 2
        frame["local_map"]["data"] = cells.tolist()
        result = detect_corridor(frame, [0, 0, math.pi / 2])
        self.assertEqual(result["status"], "insufficient_evidence")
        self.assertIsNone(result["is_narrow"])
        frame["local_map"]["data"] = np.zeros_like(cells).tolist()
        self.assertEqual(detect_corridor(frame, [0, 0, math.pi / 2])["boundaries"], {})

    def test_rotation_translation_and_row_order(self):
        frame = make_frame()
        theta = 0.6
        rotation = np.array([[np.cos(theta), -np.sin(theta)],
                             [np.sin(theta), np.cos(theta)]])
        offset = np.array([4.0, -3.0])
        grid = frame["local_map"]
        reference = obstacle_points(grid)
        grid["origin"] = (rotation @ np.array(grid["origin"]) + offset).tolist()
        grid["rotation_xyzw"] = [0, 0, np.sin(theta / 2), np.cos(theta / 2)]
        grid["data"] = grid["data"][::-1]
        grid["row_order"] = "top_to_bottom"
        np.testing.assert_allclose(obstacle_points(grid), reference @ rotation.T + offset)
        result = detect_corridor(frame, [*offset, math.pi / 2 + theta])
        self.assertEqual(result["status"], "ok")
        self.assertAlmostEqual(result["width_at_robot_m"], 1.2, delta=0.02)

    def test_large_gap_and_short_blob_rejected(self):
        s = np.r_[np.linspace(-1.5, -1.1, 50), np.linspace(1.1, 1.5, 50)]
        self.assertIsNone(fit_side(np.column_stack([s, np.full_like(s, 0.6)])))
        s = np.linspace(-0.2, 0.2, 100)
        self.assertIsNone(fit_side(np.column_stack([s, np.full_like(s, 0.6)])))


if __name__ == "__main__":
    unittest.main()
