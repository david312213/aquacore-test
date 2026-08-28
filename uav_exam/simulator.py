from __future__ import annotations

import time
from dataclasses import dataclass
from math import ceil, hypot
from typing import Any, Protocol

import numpy as np

from .geometry import CircleObstacle, RectObstacle, disk_collides
from .planner import EgoLikePlanner, sample_trajectory
from .scenario import Scenario, generate_scenario
from .types import (
    Action,
    ActionMode,
    ControllerProfile,
    GridMap,
    Observation,
    PlanStatus,
    Pose,
    RunMetrics,
    TrajectoryPoint,
)


class Controller(Protocol):
    def step(self, observation: Observation, planner: EgoLikePlanner) -> Action: ...


@dataclass(frozen=True)
class SimulatorConfig:
    dt: float = 0.1
    time_limit: float = 600.0
    body_radius: float = 0.19
    sensor_radius: float = 3.0
    grid_resolution: float = 0.1
    odom_sigma: float = 0.02
    landing_rate: float = 0.35


@dataclass
class SimulationResult:
    metrics: RunMetrics
    scenario: Scenario


class Simulator:
    def __init__(
        self,
        scenario: Scenario,
        profile: ControllerProfile,
        config: SimulatorConfig | None = None,
    ) -> None:
        self.scenario = scenario
        self.profile = profile
        self.config = config or SimulatorConfig()
        self.rng = np.random.default_rng(scenario.seed + 93_817)
        self.pose = scenario.mission.start
        self.velocity = (0.0, 0.0, 0.0)
        self.commanded_speed = 0.0
        self.time = 0.0
        self.remaining_payloads = list(scenario.mission.required_payloads)
        self.metrics = RunMetrics(scenario.seed, scenario.variant, profile.name)
        self.finished = False
        self.active_trajectory: tuple[TrajectoryPoint, ...] | None = None
        self.trajectory_elapsed = 0.0
        self.last_observed_pose = self.pose
        self.cached_grid = self._build_local_grid(self.pose, stamp=0.0)
        self.last_map_update = 0.0
        self.last_event = "simulation_started"
        self.contacting: set[int] = set()
        self.gates_seen: set[int] = set()
        self.takeoff_stable_time = 0.0
        self.has_been_airborne = False
        self.planner_call_times: list[float] = []
        self.controller_times_ms: list[float] = []
        self._last_planner_calls = 0
        self._recovery_pending = False
        self._valid_after_fault = 0

    def run(self, controller: Controller, planner: EgoLikePlanner | None = None) -> SimulationResult:
        planner = planner or EgoLikePlanner()
        planner.reset_metrics()
        if hasattr(controller, "reset"):
            controller.reset()  # type: ignore[attr-defined]
        while not self.finished and self.time < self.config.time_limit:
            observation = self._observe(planner)
            started = time.perf_counter()
            try:
                action = controller.step(observation, planner)
                if not isinstance(action, Action):
                    raise TypeError("controller.step() must return uav_exam.types.Action")
            except Exception as exc:  # An episode should yield diagnostics rather than kill a batch.
                action = Action.abort(f"controller exception: {type(exc).__name__}: {exc}")
                self._event(action.reason)
            self.controller_times_ms.append((time.perf_counter() - started) * 1000.0)
            if planner.total_calls > self._last_planner_calls:
                self.planner_call_times.extend(
                    [self.time] * (planner.total_calls - self._last_planner_calls)
                )
                self._last_planner_calls = planner.total_calls
            self._apply_action(action, observation)
            self._advance_motion(action)
            self._update_task_events()
            self.metrics.path.append((self.time, self.pose.x, self.pose.y, self.pose.z))
            self.time = round(self.time + self.config.dt, 10)

        if not self.finished:
            self._event("time_limit_reached")
            self.finished = True
        self.metrics.duration = min(self.time, self.config.time_limit)
        self.metrics.planner_calls = planner.total_calls
        self.metrics.planner_failures = planner.failures
        self.metrics.mean_planner_ms = float(np.mean(planner.compute_times_ms)) if planner.compute_times_ms else 0.0
        self.metrics.mean_controller_ms = (
            float(np.mean(self.controller_times_ms)) if self.controller_times_ms else 0.0
        )
        self.metrics.replan_compliance = self._replan_compliance()
        self.metrics.finalize_score()
        return SimulationResult(self.metrics, self.scenario)

    def _observe(self, planner: EgoLikePlanner) -> Observation:
        odom_valid = not self.scenario.fault_active("odom", self.time)
        map_valid = not self.scenario.fault_active("map", self.time)
        if odom_valid:
            self.last_observed_pose = Pose(
                self.pose.x + float(self.rng.normal(0.0, self.config.odom_sigma)),
                self.pose.y + float(self.rng.normal(0.0, self.config.odom_sigma)),
                max(0.0, self.pose.z + float(self.rng.normal(0.0, self.config.odom_sigma / 2))),
            )
        if map_valid:
            self.cached_grid = self._build_local_grid(self.last_observed_pose, stamp=self.time)
            self.last_map_update = self.time
        return Observation(
            time=self.time,
            pose=self.last_observed_pose,
            velocity=self.velocity,
            odom_valid=odom_valid,
            map_age=max(0.0, self.time - self.last_map_update),
            local_grid=self.cached_grid,
            # Retained for controller compatibility; delivery coordinates are public.
            detections=(),
            remaining_payloads=tuple(self.remaining_payloads),
            planner_status=planner.last_status,
            mission=self.scenario.mission,
            last_event=self.last_event,
        )

    def _build_local_grid(self, center: Pose, stamp: float) -> GridMap:
        resolution = self.config.grid_resolution
        half_cells = int(round(self.config.sensor_radius / resolution))
        size = 2 * half_cells + 1
        origin_x = center.x - (half_cells + 0.5) * resolution
        origin_y = center.y - (half_cells + 0.5) * resolution
        xs = origin_x + (np.arange(size) + 0.5) * resolution
        ys = origin_y + (np.arange(size) + 0.5) * resolution
        xx, yy = np.meshgrid(xs, ys)
        known = (xx - center.x) ** 2 + (yy - center.y) ** 2 <= self.config.sensor_radius**2
        occupancy = np.full((size, size), -1, dtype=np.int8)
        occupancy[known] = 0
        heights = np.zeros((size, size), dtype=np.float32)
        cell_pad = resolution / 2
        for obstacle in self.scenario.obstacles:
            if isinstance(obstacle, CircleObstacle):
                raster_radius = obstacle.radius + resolution / np.sqrt(2)
                mask = (xx - obstacle.x) ** 2 + (yy - obstacle.y) ** 2 <= raster_radius**2
            elif isinstance(obstacle, RectObstacle):
                mask = (
                    (xx >= obstacle.x_min - cell_pad)
                    & (xx <= obstacle.x_max + cell_pad)
                    & (yy >= obstacle.y_min - cell_pad)
                    & (yy <= obstacle.y_max + cell_pad)
                )
            else:  # pragma: no cover - all built-in scenarios use the two shapes.
                mask = np.zeros_like(known)
            mask &= known
            occupancy[mask] = 1
            heights[mask] = np.maximum(heights[mask], obstacle.height)
        return GridMap(occupancy, heights, origin_x, origin_y, resolution, stamp)

    def _apply_action(self, action: Action, observation: Observation) -> None:
        raw_unsafe = not observation.odom_valid or observation.map_age > 0.5
        if raw_unsafe:
            self._recovery_pending = True
            self._valid_after_fault = 0
        elif self._recovery_pending:
            self._valid_after_fault += 1
            if self._valid_after_fault >= 5:
                self._recovery_pending = False
                self._event("sensors_recovered")
        recovery_guarded = self._recovery_pending and not raw_unsafe and self._valid_after_fault < 5
        unsafe = raw_unsafe or recovery_guarded
        if unsafe:
            if action.mode in {ActionMode.HOVER, ActionMode.LAND, ActionMode.ABORT}:
                if action.mode is ActionMode.HOVER:
                    self.metrics.safe_hovers += 1
            else:
                self.metrics.safety_violations += 1
                self._event("unsafe_command_replaced_with_hover")
                action = Action.hover("invalid odometry or stale map")

        if action.mode is ActionMode.FOLLOW_TRAJECTORY:
            if action.trajectory is not None:
                if not action.trajectory:
                    self._event("empty_trajectory_rejected")
                    self.active_trajectory = None
                else:
                    self.active_trajectory = action.trajectory
                    self.trajectory_elapsed = 0.0
        elif action.mode is ActionMode.HOVER:
            self.active_trajectory = None
            self.velocity = (0.0, 0.0, 0.0)
        elif action.mode is ActionMode.DROP:
            self.active_trajectory = None
            self._drop_payload(action.payload_label)
        elif action.mode is ActionMode.LAND:
            self.active_trajectory = None
        elif action.mode is ActionMode.ABORT:
            self.active_trajectory = None
            self.metrics.aborted = True
            self.finished = True
            self._event(f"aborted: {action.reason or 'controller request'}")

        self._effective_action = action

    def _advance_motion(self, requested_action: Action) -> None:
        action = getattr(self, "_effective_action", requested_action)
        old = self.pose
        new = old
        self.commanded_speed = 0.0
        if action.mode is ActionMode.LAND:
            new = Pose(old.x, old.y, max(0.0, old.z - self.config.landing_rate * self.config.dt))
            self.commanded_speed = self.config.landing_rate
        elif action.mode is ActionMode.FOLLOW_TRAJECTORY and self.active_trajectory:
            previous_command = sample_trajectory(self.active_trajectory, self.trajectory_elapsed)
            self.trajectory_elapsed += self.config.dt
            new = sample_trajectory(self.active_trajectory, self.trajectory_elapsed)
            self.commanded_speed = previous_command.distance_3d(new) / self.config.dt
        if new.z > self.scenario.mission.arena_height + 1e-9:
            self.metrics.fatal_collision = True
            self.metrics.collision_count += 1
            self._event("ceiling_violation")
            self.finished = True
            return
        accepted = self._collision_checked_pose(old, new)
        self.pose = accepted
        self.velocity = (
            (accepted.x - old.x) / self.config.dt,
            (accepted.y - old.y) / self.config.dt,
            (accepted.z - old.z) / self.config.dt,
        )
        if action.mode is ActionMode.LAND and self.pose.z <= 1e-9:
            self._finish_landing()

    def _collision_checked_pose(self, old: Pose, new: Pose) -> Pose:
        distance = max(old.distance_3d(new), 0.001)
        samples = max(1, int(ceil(distance / 0.02)))
        last_safe = old
        current_contacts: set[int] = set()
        for index in range(1, samples + 1):
            ratio = index / samples
            sample = Pose(
                old.x + (new.x - old.x) * ratio,
                old.y + (new.y - old.y) * ratio,
                old.z + (new.z - old.z) * ratio,
            )
            blocking = False
            for obs_index, obstacle in enumerate(self.scenario.obstacles):
                if not disk_collides(obstacle, sample.x, sample.y, self.config.body_radius):
                    continue
                if sample.z > obstacle.height + 0.05:
                    self.metrics.overflight = True
                    continue
                current_contacts.add(obs_index)
                if obs_index not in self.contacting:
                    self.metrics.collision_count += 1
                    self._event(f"contact:{obstacle.kind}")
                if obstacle.kind in {"boundary", "corridor_wall"} or obstacle.kind.startswith("gate_"):
                    self.metrics.fatal_collision = True
                    self.finished = True
                    blocking = True
                    break
                self.metrics.obstacle_contact = True
                blocking = True
            if blocking:
                self.contacting = current_contacts
                return last_safe
            last_safe = sample
        self.contacting = current_contacts
        return new

    def _update_task_events(self) -> None:
        if self.finished:
            return
        if self.pose.z > 1.0:
            self.has_been_airborne = True
            self.takeoff_stable_time += self.config.dt
            if self.takeoff_stable_time >= 10.0 and not self.metrics.takeoff_awarded:
                self.metrics.takeoff_awarded = True
                self._event("takeoff_awarded")
        elif not self.metrics.takeoff_awarded:
            self.takeoff_stable_time = 0.0

        corridor_entry = self.scenario.mission.corridor_bounds[0]
        if self.pose.x >= corridor_entry and not self.metrics.avoidance_awarded:
            if not self.metrics.obstacle_contact and not self.metrics.overflight:
                self.metrics.avoidance_awarded = True
                self._event("avoidance_awarded")

        if len(self.metrics.path) >= 1:
            previous_x = self.metrics.path[-1][1]
            for gate in self.scenario.gates:
                if gate.index in self.gates_seen or not (previous_x < gate.x <= self.pose.x):
                    continue
                self.gates_seen.add(gate.index)
                low, high = gate.gap
                if self.pose.z > self.scenario.mission.obstacle_height_limit:
                    self.metrics.overflight = True
                    self._event(f"gate_{gate.index + 1}_overflown")
                    continue
                # Use the intended trajectory speed rather than the measured
                # pose delta, which also contains odometry/replan noise.
                speed = self.commanded_speed
                if speed > 0.5 + 1e-9:
                    self._event(f"gate_{gate.index + 1}_too_fast:v={speed:.2f}")
                    continue
                edge_clearance = min(self.pose.y - low, high - self.pose.y) - self.config.body_radius
                if edge_clearance >= 0.05:
                    self.metrics.door_scores[gate.index] = 10.0
                    self._event(f"gate_{gate.index + 1}_clean")
                elif edge_clearance >= 0.0:
                    self.metrics.door_scores[gate.index] = 5.0
                    self._event(f"gate_{gate.index + 1}_touch")

    def _drop_payload(self, label: str | None) -> None:
        if not label or label not in self.remaining_payloads:
            self._event("invalid_drop_request")
            return
        expected = self.remaining_payloads[0]
        if label != expected:
            self._event(f"drop_rejected_wrong_order:expected={expected}:received={label}")
            return
        zone = self.scenario.mission.delivery_zone(label)
        if not (0.8 <= self.pose.z <= 1.5):
            self._event(f"drop_rejected_altitude:{label}:z={self.pose.z:.2f}")
            return
        if not zone.contains_xy(self.pose.x, self.pose.y):
            self._event(f"drop_rejected_outside_zone:{label}")
            return
        error = hypot(self.pose.x - zone.x, self.pose.y - zone.y)
        score = 15.0
        self.remaining_payloads.pop(0)
        self.metrics.drop_errors[label] = error
        self.metrics.drop_scores[label] = score
        self.metrics.drops.append((label, self.pose.x, self.pose.y, score))
        self._event(f"payload_dropped:{label}:{score:.1f}")

    def _finish_landing(self) -> None:
        landing = self.scenario.mission.landing
        error = hypot(self.pose.x - landing.x, self.pose.y - landing.y)
        if error <= self.scenario.mission.landing_radius:
            self.metrics.landing_score = 15.0
        elif error <= self.scenario.mission.landing_radius + 0.25:
            self.metrics.landing_score = 5.0
        self.metrics.completed = (
            not self.remaining_payloads
            and self.metrics.door_passes == 2
            and self.metrics.landing_score > 0
            and not self.metrics.fatal_collision
        )
        self._event(f"landed:{self.metrics.landing_score:.1f}")
        self.finished = True

    def _event(self, message: str) -> None:
        self.last_event = message
        self.metrics.events.append(f"{self.time:.1f}s {message}")

    def _replan_compliance(self) -> float:
        if len(self.planner_call_times) < 2:
            return 0.0
        intervals = np.diff(np.asarray(self.planner_call_times))
        intervals = intervals[intervals > 1e-6]
        if len(intervals) == 0:
            return 1.0
        return float(np.mean(intervals <= 0.6 + 1e-9))


def run_episode(
    controller: Controller,
    *,
    seed: int,
    variant: str,
    profile: ControllerProfile,
    config: SimulatorConfig | None = None,
) -> SimulationResult:
    scenario = generate_scenario(seed, variant)
    simulator = Simulator(scenario, profile, config=config)
    return simulator.run(controller, EgoLikePlanner())
