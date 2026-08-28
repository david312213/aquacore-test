from __future__ import annotations

import unittest

from uav_exam.scenario import DELIVERY_ZONE_CLEARANCE, generate_scenario


class ScenarioTests(unittest.TestCase):
    def test_generation_is_deterministic(self) -> None:
        first = generate_scenario(1234, "normal")
        second = generate_scenario(1234, "normal")
        self.assertEqual(first.gates, second.gates)
        self.assertEqual(first.obstacles, second.obstacles)
        self.assertEqual(first.mission.delivery_zones, second.mission.delivery_zones)

    def test_delivery_zones_and_gate_widths(self) -> None:
        scenario = generate_scenario(1001, "tight_door")
        zones = scenario.mission.delivery_zones
        self.assertEqual(
            [(zone.label, zone.x, zone.y, zone.size) for zone in zones],
            [
                ("blue", 2.0, 2.0, 0.5),
                ("purple", 4.0, 5.0, 0.5),
                ("green", 5.5, 8.0, 0.5),
            ],
        )
        self.assertEqual(tuple(zone.label for zone in zones), scenario.mission.required_payloads)
        self.assertEqual(scenario.mission.arena_height, 5.0)
        self.assertEqual(len(scenario.gates), 2)
        self.assertTrue(all(abs(gate.gap_width - 0.8) < 1e-9 for gate in scenario.gates))
        self.assertTrue(all(obstacle.height == 5.0 for obstacle in scenario.obstacles))

    def test_obstacles_keep_delivery_zone_clearance(self) -> None:
        for variant in ("normal", "delivery_near_obstacle"):
            scenario = generate_scenario(3001, variant)
            for zone in scenario.mission.delivery_zones:
                with self.subTest(variant=variant, zone=zone.label):
                    nearest = min(
                        obstacle.signed_distance(zone.x, zone.y)
                        for obstacle in scenario.obstacles
                    )
                    self.assertGreaterEqual(nearest + 1e-9, DELIVERY_ZONE_CLEARANCE)

    def test_legacy_target_variant_is_an_alias(self) -> None:
        scenario = generate_scenario(3001, "target_near_obstacle")
        self.assertEqual(scenario.variant, "delivery_near_obstacle")

    def test_dropout_variants_expose_only_documented_faults(self) -> None:
        odom = generate_scenario(7, "odom_dropout")
        mapping = generate_scenario(7, "map_dropout")
        self.assertEqual([fault.channel for fault in odom.faults], ["odom"])
        self.assertEqual([fault.channel for fault in mapping.faults], ["map"])


if __name__ == "__main__":
    unittest.main()
