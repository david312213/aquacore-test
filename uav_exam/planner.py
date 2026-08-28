from __future__ import annotations

import heapq
import time
from math import ceil, hypot

import numpy as np

from .types import GridMap, PlanResult, PlannerParams, PlanStatus, Pose, TrajectoryPoint


class EgoLikePlanner:
    """Small educational local planner inspired by EGO-Planner's system role.

    This is deliberately *not* a port or reimplementation of EGO-Planner.  It
    uses a local occupancy grid, an A* guiding path, cubic B-spline smoothing
    and simple time allocation so freshmen can exercise the surrounding
    autonomy architecture through a stable Python API.
    """

    def __init__(self) -> None:
        self.total_calls = 0
        self.failures = 0
        self.compute_times_ms: list[float] = []
        self.last_status = PlanStatus.OK

    def reset_metrics(self) -> None:
        self.total_calls = 0
        self.failures = 0
        self.compute_times_ms.clear()
        self.last_status = PlanStatus.OK

    def plan(
        self,
        start: Pose,
        grid: GridMap,
        goal: Pose,
        params: PlannerParams,
    ) -> PlanResult:
        started = time.perf_counter()
        self.total_calls += 1
        try:
            params = params.validated()
            result = self._plan_impl(start, grid, goal, params)
        except (ValueError, IndexError, FloatingPointError) as exc:
            result = PlanResult(PlanStatus.INVALID_INPUT, message=str(exc))
        elapsed = (time.perf_counter() - started) * 1000.0
        result.compute_ms = elapsed
        self.compute_times_ms.append(elapsed)
        self.last_status = result.status
        if result.status is not PlanStatus.OK:
            self.failures += 1
        return result

    def _plan_impl(
        self,
        start: Pose,
        grid: GridMap,
        goal: Pose,
        params: PlannerParams,
    ) -> PlanResult:
        requested_start_cell = grid.world_to_cell(start.x, start.y)
        if requested_start_cell is None:
            return PlanResult(PlanStatus.INVALID_INPUT, message="start lies outside local grid")
        local_goal, reached_requested_goal = self._local_goal(start, goal, params.planning_horizon)
        goal_cell = grid.world_to_cell(local_goal.x, local_goal.y)
        if goal_cell is None:
            return PlanResult(PlanStatus.NO_PATH, message="local goal lies outside sensed grid")

        inflated = self._inflated_heights(grid, params.clearance)
        direct = self._sample_segment(start, local_goal, grid.resolution / 2)
        direct_xyz, direct_overflight = self._assign_altitudes(
            direct, inflated, grid, start.z, local_goal.z, params
        )
        if direct_xyz is not None:
            trajectory = self._time_parameterize(direct_xyz, params.max_speed)
            return PlanResult(
                PlanStatus.OK,
                trajectory,
                used_overflight=direct_overflight,
                reached_requested_goal=reached_requested_goal,
            )

        traversable = self._traversable_mask(inflated, grid, params)
        requested_goal_cell = goal_cell
        start_cell = self._nearest_traversable(requested_start_cell, traversable)
        goal_cell = self._nearest_traversable(requested_goal_cell, traversable)
        if start_cell is None or goal_cell is None:
            return PlanResult(PlanStatus.NO_PATH, message="start or goal is inside inflated obstacle")
        if reached_requested_goal and goal_cell != requested_goal_cell:
            return PlanResult(PlanStatus.NO_PATH, message="requested goal violates clearance")
        cell_path = self._astar(start_cell, goal_cell, traversable)
        if not cell_path:
            return PlanResult(PlanStatus.NO_PATH, message="no collision-free guiding path")

        points = [(start.x, start.y)]
        if start_cell != requested_start_cell:
            points.append(grid.cell_to_world(*start_cell))
        points.extend(grid.cell_to_world(row, col) for row, col in cell_path[1:-1])
        if goal_cell == requested_goal_cell:
            points.append((local_goal.x, local_goal.y))
        else:
            points.append(grid.cell_to_world(*goal_cell))
        simplified = self._simplify(points, inflated, grid, params)
        smooth_xy = self._bspline_or_polyline(simplified, inflated, grid, params)
        z0 = float(np.clip(start.z, 0.0, params.z_max))
        z1 = float(np.clip(local_goal.z, params.z_min, params.z_max))
        xyz = [
            (x, y, z0 + (z1 - z0) * index / max(1, len(smooth_xy) - 1))
            for index, (x, y) in enumerate(smooth_xy)
        ]
        trajectory = self._time_parameterize(xyz, params.max_speed)
        return PlanResult(
            PlanStatus.OK,
            trajectory,
            used_overflight=False,
            reached_requested_goal=reached_requested_goal,
        )

    @staticmethod
    def _local_goal(start: Pose, goal: Pose, horizon: float) -> tuple[Pose, bool]:
        dx, dy = goal.x - start.x, goal.y - start.y
        distance = hypot(dx, dy)
        usable_horizon = max(0.5, horizon - 0.25)
        if distance <= usable_horizon:
            return goal, True
        ratio = usable_horizon / distance
        return (
            Pose(
                start.x + dx * ratio,
                start.y + dy * ratio,
                start.z + (goal.z - start.z) * ratio,
            ),
            False,
        )

    @staticmethod
    def _inflated_heights(grid: GridMap, clearance: float) -> np.ndarray:
        base = np.where(grid.occupancy == 1, grid.heights, 0.0)
        rows, cols = base.shape
        # Cell-centre occupancy underestimates a physical boundary by up to half
        # a cell diagonal.  Include that discretisation allowance so a requested
        # 0.25 m effective radius is also safe in continuous collision checks.
        effective_clearance = clearance + grid.resolution / np.sqrt(2)
        radius = int(ceil(effective_clearance / grid.resolution))
        padded = np.pad(base, radius, mode="constant", constant_values=0.0)
        inflated = np.zeros_like(base)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if hypot(dx, dy) * grid.resolution > effective_clearance + 1e-9:
                    continue
                view = padded[
                    radius + dy : radius + dy + rows,
                    radius + dx : radius + dx + cols,
                ]
                np.maximum(inflated, view, out=inflated)
        return inflated

    @staticmethod
    def _traversable_mask(
        inflated: np.ndarray, grid: GridMap, params: PlannerParams
    ) -> np.ndarray:
        known = grid.occupancy != -1
        blocked_by_height = inflated + 0.15 > params.z_max + 1e-9
        return known & ~blocked_by_height

    @staticmethod
    def _nearest_traversable(
        cell: tuple[int, int], mask: np.ndarray, max_radius: int = 5
    ) -> tuple[int, int] | None:
        row, col = cell
        if mask[row, col]:
            return cell
        rows, cols = mask.shape
        candidates: list[tuple[float, tuple[int, int]]] = []
        for radius in range(1, max_radius + 1):
            for rr in range(max(0, row - radius), min(rows, row + radius + 1)):
                for cc in range(max(0, col - radius), min(cols, col + radius + 1)):
                    if mask[rr, cc]:
                        candidates.append((hypot(rr - row, cc - col), (rr, cc)))
            if candidates:
                return min(candidates, key=lambda item: item[0])[1]
        return None

    @staticmethod
    def _astar(
        start: tuple[int, int], goal: tuple[int, int], traversable: np.ndarray
    ) -> list[tuple[int, int]]:
        rows, cols = traversable.shape
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 2**0.5),
            (-1, 1, 2**0.5),
            (1, -1, 2**0.5),
            (1, 1, 2**0.5),
        )
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        best = {start: 0.0}
        visited: set[tuple[int, int]] = set()
        while queue:
            _, cost, current = heapq.heappop(queue)
            if current in visited:
                continue
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            visited.add(current)
            row, col = current
            for dr, dc, step_cost in neighbors:
                rr, cc = row + dr, col + dc
                if not (0 <= rr < rows and 0 <= cc < cols and traversable[rr, cc]):
                    continue
                if dr and dc and not (traversable[row + dr, col] and traversable[row, col + dc]):
                    continue
                candidate = cost + step_cost
                nxt = (rr, cc)
                if candidate >= best.get(nxt, float("inf")):
                    continue
                best[nxt] = candidate
                came_from[nxt] = current
                heuristic = hypot(goal[0] - rr, goal[1] - cc)
                heapq.heappush(queue, (candidate + heuristic, candidate, nxt))
        return []

    @staticmethod
    def _sample_segment(
        start: Pose, goal: Pose, spacing: float
    ) -> list[tuple[float, float]]:
        distance = hypot(goal.x - start.x, goal.y - start.y)
        count = max(2, int(ceil(distance / max(spacing, 0.02))) + 1)
        return [
            (
                start.x + (goal.x - start.x) * index / (count - 1),
                start.y + (goal.y - start.y) * index / (count - 1),
            )
            for index in range(count)
        ]

    @staticmethod
    def _assign_altitudes(
        xy: list[tuple[float, float]],
        inflated: np.ndarray,
        grid: GridMap,
        start_z: float,
        goal_z: float,
        params: PlannerParams,
    ) -> tuple[list[tuple[float, float, float]] | None, bool]:
        required: list[float] = []
        used_overflight = False
        for x, y in xy:
            cell = grid.world_to_cell(x, y)
            if cell is None or grid.occupancy[cell] == -1:
                return None, False
            height = float(inflated[cell])
            if height > 0.0:
                if height + 0.15 > params.z_max + 1e-9:
                    return None, False
                used_overflight = True
            required.append(height + 0.15 if height > 0 else 0.0)
        if used_overflight:
            window = max(2, int(ceil(0.35 / grid.resolution)))
            expanded = []
            for index in range(len(required)):
                expanded.append(max(required[max(0, index - window) : index + window + 1]))
            required = expanded
        result: list[tuple[float, float, float]] = []
        for index, (x, y) in enumerate(xy):
            ratio = index / max(1, len(xy) - 1)
            nominal = start_z + (goal_z - start_z) * ratio
            z = max(nominal, required[index])
            if z > params.z_max + 1e-9:
                return None, False
            result.append((x, y, max(0.0, z)))
        return result, used_overflight

    def _segment_clear(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
        inflated: np.ndarray,
        grid: GridMap,
        params: PlannerParams,
    ) -> bool:
        start = Pose(a[0], a[1], params.z_min)
        goal = Pose(b[0], b[1], params.z_min)
        samples = self._sample_segment(start, goal, grid.resolution / 2)
        for x, y in samples:
            cell = grid.world_to_cell(x, y)
            if cell is None or grid.occupancy[cell] == -1:
                return False
            if inflated[cell] > 0.0:
                return False
        return True

    def _simplify(
        self,
        points: list[tuple[float, float]],
        inflated: np.ndarray,
        grid: GridMap,
        params: PlannerParams,
    ) -> list[tuple[float, float]]:
        if len(points) <= 2:
            return points
        result = [points[0]]
        anchor = 0
        while anchor < len(points) - 1:
            candidate = len(points) - 1
            while candidate > anchor + 1:
                if self._segment_clear(points[anchor], points[candidate], inflated, grid, params):
                    break
                candidate -= 1
            result.append(points[candidate])
            anchor = candidate
        return result

    def _bspline_or_polyline(
        self,
        points: list[tuple[float, float]],
        inflated: np.ndarray,
        grid: GridMap,
        params: PlannerParams,
    ) -> list[tuple[float, float]]:
        polyline = self._densify_polyline(points, grid.resolution / 2)
        if len(points) < 3:
            return polyline
        control = np.asarray([points[0], points[0], *points, points[-1], points[-1]], dtype=float)
        samples: list[tuple[float, float]] = []
        for index in range(len(control) - 3):
            p0, p1, p2, p3 = control[index : index + 4]
            for u in np.linspace(0.0, 1.0, 7, endpoint=False):
                b0 = (1 - u) ** 3 / 6
                b1 = (3 * u**3 - 6 * u**2 + 4) / 6
                b2 = (-3 * u**3 + 3 * u**2 + 3 * u + 1) / 6
                b3 = u**3 / 6
                point = b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3
                samples.append((float(point[0]), float(point[1])))
        samples.append(points[-1])
        samples[0] = points[0]
        for point in samples:
            cell = grid.world_to_cell(*point)
            if cell is None or grid.occupancy[cell] == -1 or inflated[cell] > 0:
                return polyline
        return self._densify_polyline(samples, grid.resolution / 2)

    @staticmethod
    def _densify_polyline(
        points: list[tuple[float, float]], spacing: float
    ) -> list[tuple[float, float]]:
        result: list[tuple[float, float]] = [points[0]]
        for a, b in zip(points, points[1:]):
            distance = hypot(b[0] - a[0], b[1] - a[1])
            count = max(1, int(ceil(distance / max(spacing, 0.02))))
            for index in range(1, count + 1):
                ratio = index / count
                result.append((a[0] + (b[0] - a[0]) * ratio, a[1] + (b[1] - a[1]) * ratio))
        return result

    @staticmethod
    def _time_parameterize(
        xyz: list[tuple[float, float, float]], max_speed: float
    ) -> tuple[TrajectoryPoint, ...]:
        if not xyz:
            return ()
        points = [TrajectoryPoint(0.0, *xyz[0])]
        elapsed = 0.0
        for previous, current in zip(xyz, xyz[1:]):
            distance = float(np.linalg.norm(np.asarray(current) - np.asarray(previous)))
            elapsed += distance / max(max_speed, 0.1)
            points.append(TrajectoryPoint(elapsed, *current))
        if len(points) == 1:
            points.append(TrajectoryPoint(0.1, *xyz[0]))
        return tuple(points)


def sample_trajectory(trajectory: tuple[TrajectoryPoint, ...], elapsed: float) -> Pose:
    if not trajectory:
        raise ValueError("cannot sample an empty trajectory")
    if elapsed <= trajectory[0].t:
        return trajectory[0].pose
    if elapsed >= trajectory[-1].t:
        return trajectory[-1].pose
    for left, right in zip(trajectory, trajectory[1:]):
        if left.t <= elapsed <= right.t:
            span = max(right.t - left.t, 1e-9)
            ratio = (elapsed - left.t) / span
            return Pose(
                left.x + (right.x - left.x) * ratio,
                left.y + (right.y - left.y) * ratio,
                left.z + (right.z - left.z) * ratio,
            )
    return trajectory[-1].pose
