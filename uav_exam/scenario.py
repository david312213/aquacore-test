from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable

import numpy as np

from .geometry import CircleObstacle, Obstacle, RectObstacle, point_clear
from .types import MissionSpec, Pose


UNIFORM_OBSTACLE_HEIGHT = 5.0
DELIVERY_ZONE_CLEARANCE = 0.8


@dataclass(frozen=True)
class Gate:
    index: int
    x: float
    gap_center_y: float
    gap_width: float = 0.8
    height: float = UNIFORM_OBSTACLE_HEIGHT

    @property
    def gap(self) -> tuple[float, float]:
        return (
            self.gap_center_y - self.gap_width / 2,
            self.gap_center_y + self.gap_width / 2,
        )


@dataclass(frozen=True)
class FaultInterval:
    channel: str
    start: float
    end: float

    def active(self, t: float) -> bool:
        return self.start <= t < self.end


@dataclass
class Scenario:
    seed: int
    variant: str
    mission: MissionSpec
    obstacles: list[Obstacle]
    gates: list[Gate]
    faults: list[FaultInterval]

    def fault_active(self, channel: str, t: float) -> bool:
        return any(f.channel == channel and f.active(t) for f in self.faults)


def _fixed_walls(mission: MissionSpec, gates: Iterable[Gate]) -> list[Obstacle]:
    width, height = mission.arena_size
    wall = 0.08
    obstacle_height = mission.arena_height
    obstacles: list[Obstacle] = [
        RectObstacle(0.0, wall, 0.0, height, obstacle_height, "boundary"),
        RectObstacle(width - wall, width, 0.0, height, obstacle_height, "boundary"),
        RectObstacle(0.0, width, 0.0, wall, obstacle_height, "boundary"),
        RectObstacle(0.0, width, height - wall, height, obstacle_height, "boundary"),
    ]
    x_min, x_max, y_min, y_max = mission.corridor_bounds
    obstacles.extend(
        [
            RectObstacle(
                x_min,
                x_max,
                y_min - wall,
                y_min + wall,
                obstacle_height,
                "corridor_wall",
            ),
            RectObstacle(
                x_min,
                x_max,
                y_max - wall,
                y_max + wall,
                obstacle_height,
                "corridor_wall",
            ),
        ]
    )
    for gate in gates:
        low, high = gate.gap
        obstacles.extend(
            [
                RectObstacle(
                    gate.x - wall / 2,
                    gate.x + wall / 2,
                    y_min,
                    low,
                    gate.height,
                    f"gate_{gate.index}",
                ),
                RectObstacle(
                    gate.x - wall / 2,
                    gate.x + wall / 2,
                    high,
                    y_max,
                    gate.height,
                    f"gate_{gate.index}",
                ),
            ]
        )
    return obstacles


def _sample_obstacles(
    rng: np.random.Generator,
    existing: list[Obstacle],
    mission: MissionSpec,
    variant: str,
) -> list[Obstacle]:
    result: list[Obstacle] = []
    protected = [(0.8, 5.0, 0.9), (6.1, 5.0, 0.8)]
    if variant == "delivery_near_obstacle":
        purple = mission.delivery_zone("purple")
        nearby = RectObstacle(
            purple.x + 0.82,
            purple.x + 1.62,
            purple.y - 0.4,
            purple.y + 0.4,
            mission.arena_height,
            "box",
        )
        nearby_x, nearby_y = nearby.center
        if not point_clear(
            existing,
            nearby_x,
            nearby_y,
            0.4,
            ignore_kinds={"boundary"},
        ):
            raise RuntimeError("unable to place deterministic delivery-zone obstacle")
        result.append(nearby)

    for index in range(len(result), 8):
        for _ in range(1000):
            x = float(rng.uniform(1.7, 5.9))
            y = float(rng.uniform(0.9, 9.1))
            if any(hypot(x - px, y - py) < radius for px, py, radius in protected):
                continue
            candidate: Obstacle
            if index < 4:
                candidate = RectObstacle(
                    x - 0.4,
                    x + 0.4,
                    y - 0.4,
                    y + 0.4,
                    mission.arena_height,
                    "box",
                )
                spacing = 0.75
            else:
                candidate = CircleObstacle(
                    x,
                    y,
                    0.25,
                    mission.arena_height,
                    "tree",
                )
                spacing = 0.6
            if any(
                candidate.signed_distance(zone.x, zone.y) < DELIVERY_ZONE_CLEARANCE - 1e-9
                for zone in mission.delivery_zones
            ):
                continue
            if point_clear(
                existing + result,
                x,
                y,
                spacing,
                ignore_kinds={"boundary", "corridor_wall"},
            ):
                result.append(candidate)
                break
        else:
            raise RuntimeError("unable to place obstacles for deterministic scenario")
    return result


def generate_scenario(seed: int, variant: str = "normal") -> Scenario:
    if variant == "target_near_obstacle":
        variant = "delivery_near_obstacle"
    allowed = {
        "normal",
        "tight_door",
        "delivery_near_obstacle",
        "odom_dropout",
        "map_dropout",
    }
    if variant not in allowed:
        raise ValueError(f"unknown scenario variant {variant!r}; expected one of {sorted(allowed)}")
    rng = np.random.default_rng(seed)
    mission = MissionSpec()
    if variant == "tight_door":
        centers = (4.67, 5.33) if seed % 2 == 0 else (5.33, 4.67)
    else:
        centers = tuple(float(rng.uniform(4.72, 5.28)) for _ in range(2))
    gates = [
        Gate(index=0, x=mission.gate_xs[0], gap_center_y=centers[0]),
        Gate(index=1, x=mission.gate_xs[1], gap_center_y=centers[1]),
    ]
    fixed = _fixed_walls(mission, gates)
    obstacles = fixed + _sample_obstacles(rng, fixed, mission, variant)
    faults: list[FaultInterval] = []
    if variant == "odom_dropout":
        faults.append(FaultInterval("odom", 15.0, 15.8))
    elif variant == "map_dropout":
        faults.append(FaultInterval("map", 18.0, 18.9))
    return Scenario(seed, variant, mission, obstacles, gates, faults)
