from __future__ import annotations

import unittest

import numpy as np

from uav_exam.planner import EgoLikePlanner
from uav_exam.types import GridMap, PlannerParams, PlanStatus, Pose


def free_grid() -> GridMap:
    occupancy = np.zeros((61, 61), dtype=np.int8)
    heights = np.zeros((61, 61), dtype=np.float32)
    return GridMap(occupancy, heights, -3.05, -3.05, 0.1, 0.0)


class PlannerTests(unittest.TestCase):
    def test_direct_plan_is_time_parameterized(self) -> None:
        planner = EgoLikePlanner()
        result = planner.plan(
            Pose(0.0, 0.0, 1.1),
            free_grid(),
            Pose(2.0, 0.0, 1.1),
            PlannerParams(),
        )
        self.assertEqual(result.status, PlanStatus.OK)
        self.assertGreater(len(result.trajectory), 2)
        self.assertTrue(all(a.t <= b.t for a, b in zip(result.trajectory, result.trajectory[1:])))
        self.assertFalse(result.used_overflight)

    def test_tall_wall_has_no_path(self) -> None:
        grid = free_grid()
        grid.occupancy[:, 35:37] = 1
        grid.heights[:, 35:37] = 5.0
        result = EgoLikePlanner().plan(
            Pose(0.0, 0.0, 1.1),
            grid,
            Pose(2.0, 0.0, 1.1),
            PlannerParams(z_max=5.0),
        )
        self.assertEqual(result.status, PlanStatus.NO_PATH)

    def test_planner_accepts_five_metre_ceiling_and_rejects_above(self) -> None:
        valid = EgoLikePlanner().plan(
            Pose(0.0, 0.0, 1.1),
            free_grid(),
            Pose(2.0, 0.0, 4.8),
            PlannerParams(z_max=5.0),
        )
        invalid = EgoLikePlanner().plan(
            Pose(0.0, 0.0, 1.1),
            free_grid(),
            Pose(2.0, 0.0, 1.1),
            PlannerParams(z_max=5.01),
        )
        self.assertEqual(valid.status, PlanStatus.OK)
        self.assertLessEqual(max(point.z for point in valid.trajectory), 5.0)
        self.assertEqual(invalid.status, PlanStatus.INVALID_INPUT)

    def test_final_goal_inside_clearance_is_rejected(self) -> None:
        grid = free_grid()
        cell = grid.world_to_cell(1.0, 0.0)
        assert cell is not None
        grid.occupancy[cell] = 1
        grid.heights[cell] = 2.0
        result = EgoLikePlanner().plan(
            Pose(0.0, 0.0, 1.1),
            grid,
            Pose(1.0, 0.0, 1.1),
            PlannerParams(),
        )
        self.assertEqual(result.status, PlanStatus.NO_PATH)


if __name__ == "__main__":
    unittest.main()
