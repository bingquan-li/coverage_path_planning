import heapq
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from astar_frame import astar, build_search_map, inflate_costs, plan_frame


class AstarFrameTest(unittest.TestCase):
    def test_circular_linear_inflation_and_class_filter(self):
        cells = np.zeros((25, 25), dtype=int)
        cells[12, 12] = 1
        cells[0, 0] = 2
        costs = inflate_costs(cells, 0.1)
        self.assertEqual(costs[12, 12], 255)
        self.assertAlmostEqual(costs[12, 17], 157.5)
        self.assertAlmostEqual(costs[12, 22], 60)
        self.assertAlmostEqual(costs[18, 20], 60)  # sqrt(6^2+8^2)*0.1 = 1 m
        self.assertEqual(costs[20, 20], 0)  # outside circle, inside bounding square
        self.assertEqual(costs[0, 0], 0)
        self.assertEqual(inflate_costs(np.full((3, 3), 2), 0.1).sum(), 0)

    def test_overlap_takes_maximum(self):
        cells = np.zeros((25, 25), dtype=int)
        cells[12, 8] = cells[12, 16] = 1
        costs = inflate_costs(cells, 0.1)
        self.assertAlmostEqual(costs[12, 12], 177)

    def test_blocked_endpoints_and_diagonal_corner(self):
        self.assertEqual(astar(np.array([[0, 255], [255, 0]]), (0, 0), (1, 1), 1), [])
        self.assertEqual(astar(np.array([[255, 0]]), (0, 0), (0, 1), 1), [])
        self.assertEqual(astar(np.array([[0, 255]]), (0, 0), (0, 1), 1), [])
        self.assertEqual(astar(np.zeros((1, 1)), (0, 0), (0, 0), 1), [(0, 0)])

    def test_cost_aware_detour_matches_dijkstra(self):
        costs = np.zeros((7, 9))
        costs[2:5, 3:6] = 240
        start, goal = (3, 0), (3, 8)
        path = astar(costs, start, goal, 1, 5)
        self.assertTrue(any(r < 2 or r > 4 for r, c in path))
        actual = sum(math.dist(a, b) * (1 + 5 * (costs[a] + costs[b]) / 510)
                     for a, b in zip(path, path[1:]))
        # Independent shortest-path reference without a heuristic.
        best = {start: 0}
        queue = [(0, start)]
        while queue:
            score, node = heapq.heappop(queue)
            if score > best[node]:
                continue
            if node == goal:
                break
            for r in range(max(0, node[0] - 1), min(7, node[0] + 2)):
                for c in range(max(0, node[1] - 1), min(9, node[1] + 2)):
                    other = (r, c)
                    candidate = score + math.dist(node, other) * (
                        1 + 5 * (costs[node] + costs[other]) / 510)
                    if candidate < best.get(other, math.inf):
                        best[other] = candidate
                        heapq.heappush(queue, (candidate, other))
        self.assertAlmostEqual(actual, best[goal])

    def test_rotated_map_top_row_first_and_exterior_inflation(self):
        grid = dict(width=2, height=2, data=[[0, 0], [1, 0]],
                    map_scale=0.5, origin=[10, 20], row_order="top_to_bottom",
                    rotation_xyzw=[0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
        sm = build_search_map(grid, [9.75, 20.25], [9.75, 17.25])
        obstacle = sm.world_to_cell([9.75, 20.25])
        self.assertEqual(sm.costs[obstacle], 255)
        np.testing.assert_allclose(sm.cell_to_world(obstacle), [9.75, 20.25])
        outside = sm.world_to_cell([10.25, 20.25])
        self.assertFalse(sm.known[outside])
        self.assertAlmostEqual(sm.costs[outside], 157.5)

    def test_real_frame_path_geometry_and_collision(self):
        with (Path(__file__).resolve().parents[1] / "frame.json").open() as source:
            frame = json.load(source)
        result, sm = plan_frame(frame)
        self.assertEqual(result["status"], "ok")
        start, goal = np.array(result["start"]), np.array(result["goal"])
        yaw = result["robot_pose"][2]
        np.testing.assert_allclose(goal - start, -3 * np.array([math.cos(yaw), math.sin(yaw)]))
        np.testing.assert_allclose(result["path"][0], start)
        np.testing.assert_allclose(result["path"][-1], goal)
        self.assertGreater(result["outside_map_path_cells"], 0)
        # Sample complete path edges, including precise start/goal connectors.
        for a, b in zip(result["path"], result["path"][1:]):
            for point in np.linspace(a, b, 5):
                self.assertLess(sm.costs[sm.world_to_cell(point)], 255)


if __name__ == "__main__":
    unittest.main()
