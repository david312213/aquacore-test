from __future__ import annotations

from .planner import EgoLikePlanner
from .types import Action, ControllerProfile, Observation, PlanStatus, Pose


class BaselineController:
    """Runnable demonstration that deliberately does not finish the mission."""

    def __init__(self, profile: ControllerProfile) -> None:
        self.profile = profile
        self.reset()

    def reset(self) -> None:
        self.state = "TAKEOFF"
        self.last_plan_time = -1e9
        self.airborne_since: float | None = None
        self.waypoint_index = 0
        self.recovery_frames = 0
        self.sensor_fault = False
        self.search_waypoints = self._coverage_waypoints()

    @staticmethod
    def _coverage_waypoints() -> list[Pose]:
        points: list[Pose] = []
        for index, y in enumerate((1.0, 2.6, 4.2, 5.8, 7.4, 9.0)):
            xs = (1.1, 6.0) if index % 2 == 0 else (6.0, 1.1)
            points.extend(Pose(x, y, 1.2) for x in xs)
        return points

    def step(self, observation: Observation, planner: EgoLikePlanner) -> Action:
        safe = observation.odom_valid and observation.map_age <= 0.5
        if not safe:
            self.sensor_fault = True
            self.recovery_frames = 0
            return Action.hover("sensor invalid")
        if self.sensor_fault:
            self.recovery_frames += 1
            if self.recovery_frames < 5:
                return Action.hover("waiting for five valid frames")
            self.sensor_fault = False

        if self.state == "TAKEOFF":
            if observation.pose.z > 1.0:
                self.airborne_since = self.airborne_since or observation.time
                if observation.time - self.airborne_since >= 10.1:
                    self.state = "SEARCH"
            return self._navigate(observation, planner, Pose(0.8, 5.0, 1.2))

        waypoint = self.search_waypoints[self.waypoint_index]
        if observation.pose.distance_xy(waypoint) < 0.22:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.search_waypoints)
            waypoint = self.search_waypoints[self.waypoint_index]
        return self._navigate(observation, planner, waypoint)

    def _navigate(
        self, observation: Observation, planner: EgoLikePlanner, goal: Pose
    ) -> Action:
        if observation.time - self.last_plan_time < 0.5:
            return Action.follow()
        self.last_plan_time = observation.time
        result = planner.plan(
            observation.pose,
            observation.local_grid,
            goal,
            self.profile.planner_params(),
        )
        return Action.follow(result) if result.status is PlanStatus.OK else Action.hover(result.message)
