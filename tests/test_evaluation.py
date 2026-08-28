from __future__ import annotations

import unittest

from uav_exam.evaluation import aggregate_runs, load_cases


class EvaluationTests(unittest.TestCase):
    def test_public_case_count_and_failure_coverage(self) -> None:
        cases = load_cases("cases/public_cases.json")
        self.assertEqual(len(cases), 20)
        variants = {case.variant for case in cases}
        self.assertEqual(
            variants,
            {
                "normal",
                "tight_door",
                "delivery_near_obstacle",
                "odom_dropout",
                "map_dropout",
            },
        )

    def test_aggregation(self) -> None:
        run = {
            "raw_score": 100.0,
            "completed": True,
            "collision_free": True,
            "fatal_collision": False,
            "door_passes": 2,
            "drop_errors": {"blue": 0.1},
            "duration": 80.0,
            "mean_controller_ms": 1.0,
            "mean_planner_ms": 2.0,
            "replan_compliance": 1.0,
            "safety_violations": 0,
        }
        summary = aggregate_runs([run, run])
        self.assertEqual(summary["average_score"], 100.0)
        self.assertEqual(summary["completion_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
