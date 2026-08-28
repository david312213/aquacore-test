from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from uav_exam.scenario import generate_scenario
from uav_exam.simulator import SimulationResult
from uav_exam.types import RunMetrics
from uav_exam.visualize import animate_run, save_run_plot


class VisualizationTests(unittest.TestCase):
    @staticmethod
    def sample_result() -> SimulationResult:
        scenario = generate_scenario(1001)
        metrics = RunMetrics(seed=1001, variant="normal", profile="adaptive")
        metrics.path = [
            (0.0, 0.8, 5.0, 0.0),
            (0.5, 0.8, 5.0, 1.1),
            (1.0, 2.0, 2.0, 1.1),
        ]
        metrics.drop_scores["blue"] = 15.0
        metrics.drops.append(("blue", 2.0, 2.0, 15.0))
        metrics.finalize_score()
        return SimulationResult(metrics, scenario)

    def test_static_two_and_three_dimensional_plots(self) -> None:
        result = self.sample_result()
        with tempfile.TemporaryDirectory() as directory:
            for view in ("2d", "3d"):
                output = Path(directory) / f"plot-{view}.png"
                save_run_plot(result, output, view=view)
                self.assertGreater(output.stat().st_size, 1_000)

    def test_three_dimensional_gif(self) -> None:
        result = self.sample_result()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "flight-3d.gif"
            animate_run(result, save_path=output, view="3d")
            self.assertGreater(output.stat().st_size, 1_000)


if __name__ == "__main__":
    unittest.main()
