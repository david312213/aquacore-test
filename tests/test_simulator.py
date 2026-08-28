from __future__ import annotations

import unittest

from uav_exam.baseline_controller import BaselineController
from uav_exam.io_utils import load_profile
from uav_exam.planner import EgoLikePlanner
from uav_exam.scenario import generate_scenario
from uav_exam.simulator import Simulator, SimulatorConfig
from uav_exam.types import Action, PlanResult, PlanStatus, Pose, TrajectoryPoint


class NeverSafeController:
    def __init__(self) -> None:
        self.was_invalid = False

    def step(self, observation, planner):
        if not observation.odom_valid:
            self.was_invalid = True
            return Action.hover("raw fault")
        if self.was_invalid:
            return Action.follow()  # Violates the five-valid-frame guard.
        return Action.hover()


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = load_profile("adaptive")

    def test_starter_baseline_does_not_complete_the_mission(self) -> None:
        scenario = generate_scenario(1001, "normal")
        simulator = Simulator(
            scenario,
            self.profile,
            SimulatorConfig(time_limit=30.0),
        )
        metrics = simulator.run(BaselineController(self.profile)).metrics
        self.assertFalse(metrics.completed)
        self.assertEqual(metrics.drop_scores, {})

    def test_recovery_guard_is_enforced_for_five_frames(self) -> None:
        scenario = generate_scenario(4001, "odom_dropout")
        simulator = Simulator(
            scenario,
            self.profile,
            SimulatorConfig(time_limit=17.0),
        )
        metrics = simulator.run(NeverSafeController()).metrics
        self.assertGreaterEqual(metrics.safety_violations, 4)

    def test_drop_requires_order_zone_and_altitude(self) -> None:
        scenario = generate_scenario(1001, "normal")
        simulator = Simulator(scenario, self.profile)
        blue = scenario.mission.delivery_zone("blue")
        purple = scenario.mission.delivery_zone("purple")

        simulator.pose = Pose(purple.x, purple.y, 1.1)
        simulator._drop_payload("purple")
        self.assertEqual(simulator.remaining_payloads[0], "blue")
        self.assertIn("drop_rejected_wrong_order", simulator.metrics.events[-1])

        simulator.pose = Pose(blue.x + blue.half_size + 0.001, blue.y, 1.1)
        simulator._drop_payload("blue")
        self.assertEqual(simulator.remaining_payloads[0], "blue")
        self.assertIn("drop_rejected_outside_zone", simulator.metrics.events[-1])

        simulator.pose = Pose(blue.x, blue.y, 1.501)
        simulator._drop_payload("blue")
        self.assertEqual(simulator.remaining_payloads[0], "blue")
        self.assertIn("drop_rejected_altitude", simulator.metrics.events[-1])

        simulator.pose = Pose(blue.x + blue.half_size, blue.y - blue.half_size, 0.8)
        simulator._drop_payload("blue")
        self.assertEqual(simulator.remaining_payloads[0], "purple")
        self.assertEqual(simulator.metrics.drop_scores["blue"], 15.0)

    def test_observation_keeps_empty_detection_field(self) -> None:
        simulator = Simulator(generate_scenario(1001), self.profile)
        observation = simulator._observe(EgoLikePlanner())
        self.assertEqual(observation.detections, ())

    def test_ceiling_violation_is_fatal_above_five_metres(self) -> None:
        simulator = Simulator(generate_scenario(1001), self.profile)
        trajectory = (
            TrajectoryPoint(0.0, simulator.pose.x, simulator.pose.y, simulator.pose.z),
            TrajectoryPoint(0.1, simulator.pose.x, simulator.pose.y, 5.01),
        )
        action = Action.follow(PlanResult(PlanStatus.OK, trajectory))
        simulator.active_trajectory = trajectory
        simulator.trajectory_elapsed = 0.0
        simulator._advance_motion(action)
        self.assertTrue(simulator.metrics.fatal_collision)
        self.assertTrue(simulator.finished)

    def test_door_score_requires_speed_at_or_below_half_metre_per_second(self) -> None:
        scenario = generate_scenario(1001)
        gate = scenario.gates[0]
        gate_center = sum(gate.gap) / 2.0

        too_fast = Simulator(scenario, self.profile)
        too_fast.metrics.path.append((0.0, gate.x - 0.01, gate_center, 1.2))
        too_fast.pose = Pose(gate.x + 0.01, gate_center, 1.2)
        too_fast.commanded_speed = 0.5001
        too_fast._update_task_events()
        self.assertEqual(too_fast.metrics.door_scores[0], 0.0)
        self.assertIn("gate_1_too_fast", too_fast.metrics.events[-1])

        compliant = Simulator(scenario, self.profile)
        compliant.metrics.path.append((0.0, gate.x - 0.01, gate_center, 1.2))
        compliant.pose = Pose(gate.x + 0.01, gate_center, 1.2)
        compliant.commanded_speed = 0.5
        compliant._update_task_events()
        self.assertEqual(compliant.metrics.door_scores[0], 10.0)


if __name__ == "__main__":
    unittest.main()
