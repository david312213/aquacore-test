"""Student submission file.

Only this file and ``configs/adaptive.json`` should be edited for the practical
part.  The starter is runnable, but intentionally implements only takeoff and
navigation to the first public delivery zone.  Ordered drops, doors, landing
and robust recovery are TODOs for students.
"""

from __future__ import annotations

from uav_exam import Action, ControllerProfile, EgoLikePlanner, Observation, PlanStatus, Pose


class StudentController:
    def __init__(self, profile: ControllerProfile) -> None:
        self.profile = profile
        self.reset()

    def reset(self) -> None:
        self.state = "TAKEOFF"
        self.last_plan_time = -1e9
        self.airborne_since: float | None = None
        self.delivery_index = 0
        self.sensor_fault = False
        self.valid_recovery_frames = 0

        # TODO 1: add the full ordered-delivery mission state.
        # TODO 2: add corridor-entry/door-scan/pass sub-states and recovery counters.

    def step(self, observation: Observation, planner: EgoLikePlanner) -> Action:
        if not observation.odom_valid or observation.map_age > 0.5:
            self.sensor_fault = True
            self.valid_recovery_frames = 0
            return Action.hover("invalid odometry or stale map")
        if self.sensor_fault:
            self.valid_recovery_frames += 1
            if self.valid_recovery_frames < 5:
                return Action.hover("waiting for five consecutive valid frames")
            self.sensor_fault = False

        # TODO 3: replace this starter with all required delivery and door states.
        # ``observation.mission.delivery_zones`` exposes blue, purple and green.
        if self.state == "TAKEOFF":
            if observation.pose.z > 1.0:
                self.airborne_since = self.airborne_since or observation.time
                if observation.time - self.airborne_since >= 10.1:
                    self.state = "APPROACH_BLUE"
            return self._navigate(observation, planner, Pose(0.8, 5.0, 1.2))

        zone = observation.mission.delivery_zones[self.delivery_index]
        goal = Pose(zone.x, zone.y, 1.2)
        if zone.contains_xy(observation.pose.x, observation.pose.y):
            # TODO 4: issue the matching DROP and advance only after confirmation.
            return Action.hover("first delivery zone reached; DROP is a student TODO")
        return self._navigate(observation, planner, goal)

    def _navigate(
        self,
        observation: Observation,
        planner: EgoLikePlanner,
        goal: Pose,
        *,
        door: bool = False,
    ) -> Action:
        if observation.time - self.last_plan_time < 0.5:
            return Action.follow()
        self.last_plan_time = observation.time
        result = planner.plan(
            observation.pose,
            observation.local_grid,
            goal,
            self.profile.planner_params(door=door),
        )
        if result.status is not PlanStatus.OK:
            # TODO 5: add bounded retry, scan recovery and safe termination.
            return Action.hover(result.message)
        return Action.follow(result)
