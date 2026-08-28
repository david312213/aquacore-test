from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import hypot
from typing import Any, Optional

import numpy as np


MISSION_MAX_SCORE = 100.0


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0

    def distance_xy(self, other: "Pose") -> float:
        return hypot(self.x - other.x, self.y - other.y)

    def distance_3d(self, other: "Pose") -> float:
        return float(
            np.linalg.norm(
                np.array((self.x - other.x, self.y - other.y, self.z - other.z))
            )
        )


@dataclass(frozen=True)
class DeliveryZone:
    """A public, axis-aligned square delivery area on the ground plane."""

    label: str
    x: float
    y: float
    size: float = 0.5

    @property
    def half_size(self) -> float:
        return self.size / 2.0

    def contains_xy(self, x: float, y: float) -> bool:
        return (
            abs(x - self.x) <= self.half_size + 1e-9
            and abs(y - self.y) <= self.half_size + 1e-9
        )


@dataclass(frozen=True)
class MissionSpec:
    arena_size: tuple[float, float] = (10.0, 10.0)
    arena_height: float = 5.0
    start: Pose = Pose(0.8, 5.0, 0.0)
    landing: Pose = Pose(9.35, 5.0, 0.0)
    landing_radius: float = 0.45
    delivery_bounds: tuple[float, float, float, float] = (0.5, 6.45, 0.5, 9.5)
    corridor_bounds: tuple[float, float, float, float] = (6.5, 9.85, 4.25, 5.75)
    gate_xs: tuple[float, float] = (7.4, 8.4)
    gate_width: float = 0.8
    obstacle_height_limit: float = 1.45
    delivery_zones: tuple[DeliveryZone, DeliveryZone, DeliveryZone] = (
        DeliveryZone("blue", 2.0, 2.0),
        DeliveryZone("purple", 4.0, 5.0),
        DeliveryZone("green", 5.5, 8.0),
    )
    required_payloads: tuple[str, str, str] = (
        "blue",
        "purple",
        "green",
    )

    def delivery_zone(self, label: str) -> DeliveryZone:
        try:
            return next(zone for zone in self.delivery_zones if zone.label == label)
        except StopIteration as exc:
            raise KeyError(f"unknown delivery zone {label!r}") from exc


@dataclass
class GridMap:
    """A local world-aligned occupancy grid.

    Occupancy uses -1 for unknown, 0 for free and 1 for occupied.  ``heights``
    stores the highest observed obstacle at each occupied cell.
    """

    occupancy: np.ndarray
    heights: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float
    stamp: float

    @property
    def shape(self) -> tuple[int, int]:
        return self.occupancy.shape

    def world_to_cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        rows, cols = self.occupancy.shape
        if 0 <= row < rows and 0 <= col < cols:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def sample(self, x: float, y: float) -> tuple[int, float]:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return -1, 0.0
        row, col = cell
        return int(self.occupancy[row, col]), float(self.heights[row, col])


@dataclass(frozen=True)
class TargetDetection:
    label: str
    relative_x: float
    relative_y: float
    confidence: float
    stamp: float


class PlanStatus(str, Enum):
    OK = "OK"
    NO_PATH = "NO_PATH"
    INVALID_INPUT = "INVALID_INPUT"


@dataclass(frozen=True)
class PlannerParams:
    max_speed: float = 0.7
    max_accel: float = 1.0
    clearance: float = 0.25
    z_min: float = 0.8
    z_max: float = 1.45
    planning_horizon: float = 3.0

    def validated(self) -> "PlannerParams":
        if not (0.1 <= self.max_speed <= 2.0):
            raise ValueError("max_speed must be in [0.1, 2.0] m/s")
        if not (0.1 <= self.max_accel <= 4.0):
            raise ValueError("max_accel must be in [0.1, 4.0] m/s^2")
        if not (0.19 <= self.clearance <= 0.5):
            raise ValueError("clearance must be in [0.19, 0.5] m")
        if not (0.0 <= self.z_min < self.z_max <= 5.0):
            raise ValueError("invalid altitude range")
        if not (1.0 <= self.planning_horizon <= 5.0):
            raise ValueError("planning_horizon must be in [1.0, 5.0] m")
        return self


@dataclass(frozen=True)
class TrajectoryPoint:
    t: float
    x: float
    y: float
    z: float

    @property
    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.z)


@dataclass
class PlanResult:
    status: PlanStatus
    trajectory: tuple[TrajectoryPoint, ...] = ()
    compute_ms: float = 0.0
    message: str = ""
    used_overflight: bool = False
    reached_requested_goal: bool = False


class ActionMode(str, Enum):
    FOLLOW_TRAJECTORY = "FOLLOW_TRAJECTORY"
    HOVER = "HOVER"
    DROP = "DROP"
    LAND = "LAND"
    ABORT = "ABORT"


@dataclass(frozen=True)
class Action:
    mode: ActionMode
    trajectory: Optional[tuple[TrajectoryPoint, ...]] = None
    payload_label: Optional[str] = None
    reason: str = ""

    @classmethod
    def follow(cls, result: Optional[PlanResult] = None) -> "Action":
        if result is None:
            return cls(ActionMode.FOLLOW_TRAJECTORY, trajectory=None)
        if result.status is not PlanStatus.OK:
            raise ValueError("cannot follow an unsuccessful plan")
        return cls(ActionMode.FOLLOW_TRAJECTORY, trajectory=result.trajectory)

    @classmethod
    def hover(cls, reason: str = "") -> "Action":
        return cls(ActionMode.HOVER, reason=reason)

    @classmethod
    def drop(cls, payload_label: str) -> "Action":
        return cls(ActionMode.DROP, payload_label=payload_label)

    @classmethod
    def land(cls, reason: str = "") -> "Action":
        return cls(ActionMode.LAND, reason=reason)

    @classmethod
    def abort(cls, reason: str = "") -> "Action":
        return cls(ActionMode.ABORT, reason=reason)


@dataclass(frozen=True)
class Observation:
    time: float
    pose: Pose
    velocity: tuple[float, float, float]
    odom_valid: bool
    map_age: float
    local_grid: GridMap
    detections: tuple[TargetDetection, ...]
    remaining_payloads: tuple[str, ...]
    planner_status: PlanStatus
    mission: MissionSpec
    last_event: str = ""


@dataclass(frozen=True)
class ControllerProfile:
    name: str
    max_speed: float
    max_accel: float
    clearance: float
    z_max: float
    planning_horizon: float = 3.0
    door_speed: float = 0.5
    door_clearance: float = 0.22

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ControllerProfile":
        return cls(**data)

    def planner_params(self, *, door: bool = False) -> PlannerParams:
        return PlannerParams(
            max_speed=min(self.door_speed, 0.5) if door else self.max_speed,
            max_accel=self.max_accel,
            clearance=self.door_clearance if door else self.clearance,
            z_min=0.8,
            z_max=min(self.z_max, 1.45) if door else self.z_max,
            planning_horizon=self.planning_horizon,
        ).validated()


@dataclass
class RunMetrics:
    seed: int
    variant: str
    profile: str
    raw_score: float = 0.0
    completed: bool = False
    aborted: bool = False
    duration: float = 0.0
    takeoff_awarded: bool = False
    avoidance_awarded: bool = False
    landing_score: float = 0.0
    door_scores: list[float] = field(default_factory=lambda: [0.0, 0.0])
    drop_scores: dict[str, float] = field(default_factory=dict)
    drop_errors: dict[str, float] = field(default_factory=dict)
    collision_count: int = 0
    fatal_collision: bool = False
    obstacle_contact: bool = False
    overflight: bool = False
    planner_calls: int = 0
    planner_failures: int = 0
    replan_compliance: float = 0.0
    mean_planner_ms: float = 0.0
    mean_controller_ms: float = 0.0
    safety_violations: int = 0
    safe_hovers: int = 0
    events: list[str] = field(default_factory=list)
    path: list[tuple[float, float, float, float]] = field(default_factory=list)
    drops: list[tuple[str, float, float, float]] = field(default_factory=list)

    @property
    def door_passes(self) -> int:
        return sum(score > 0 for score in self.door_scores)

    @property
    def collision_free(self) -> bool:
        return self.collision_count == 0

    def finalize_score(self) -> float:
        self.raw_score = round(
            (10.0 if self.takeoff_awarded else 0.0)
            + (10.0 if self.avoidance_awarded else 0.0)
            + sum(self.drop_scores.values())
            + sum(self.door_scores)
            + self.landing_score,
            2,
        )
        if self.raw_score > MISSION_MAX_SCORE + 1e-9:
            raise ValueError(f"mission score exceeded {MISSION_MAX_SCORE:g}")
        return self.raw_score

    def to_dict(self, *, include_path: bool = False) -> dict[str, Any]:
        data = asdict(self)
        data["door_passes"] = self.door_passes
        data["collision_free"] = self.collision_free
        if not include_path:
            data.pop("path", None)
        return data
