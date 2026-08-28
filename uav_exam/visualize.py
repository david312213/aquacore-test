from __future__ import annotations

from pathlib import Path

import numpy as np

from .geometry import CircleObstacle, RectObstacle
from .scenario import Scenario
from .simulator import SimulationResult


OBSTACLE_COLORS = {
    "box": "#a16207",
    "tree": "#15803d",
    "boundary": "#111827",
    "corridor_wall": "#475569",
}
DELIVERY_COLORS = {
    "blue": "#2563eb",
    "purple": "#9333ea",
    "green": "#16a34a",
}
VALID_VIEWS = {"2d", "3d"}


def _obstacle_color(kind: str) -> str:
    return OBSTACLE_COLORS.get(kind, "#7c3aed" if kind.startswith("gate_") else "#64748b")


def _validate_view(view: str) -> str:
    normalized = view.lower()
    if normalized not in VALID_VIEWS:
        raise ValueError(f"view must be one of {sorted(VALID_VIEWS)}")
    return normalized


def _draw_scene_2d(axis, scenario: Scenario) -> None:
    import matplotlib.patches as patches

    for obstacle in scenario.obstacles:
        color = _obstacle_color(obstacle.kind)
        if isinstance(obstacle, CircleObstacle):
            patch = patches.Circle((obstacle.x, obstacle.y), obstacle.radius, color=color, alpha=0.65)
        else:
            patch = patches.Rectangle(
                (obstacle.x_min, obstacle.y_min),
                obstacle.x_max - obstacle.x_min,
                obstacle.y_max - obstacle.y_min,
                color=color,
                alpha=0.72,
            )
        axis.add_patch(patch)

    for zone in scenario.mission.delivery_zones:
        color = DELIVERY_COLORS[zone.label]
        patch = patches.Rectangle(
            (zone.x - zone.half_size, zone.y - zone.half_size),
            zone.size,
            zone.size,
            facecolor=color,
            edgecolor="#111827",
            linewidth=1.2,
            alpha=0.82,
            zorder=4,
        )
        axis.add_patch(patch)
        axis.text(zone.x, zone.y, zone.label, color="white", fontsize=7, ha="center", va="center", zorder=5)

    mission = scenario.mission
    axis.scatter(mission.start.x, mission.start.y, marker="^", s=100, color="#f59e0b", label="start")
    landing = patches.Circle(
        (mission.landing.x, mission.landing.y),
        mission.landing_radius,
        fill=False,
        lw=2,
        ec="#16a34a",
        label="landing zone",
    )
    axis.add_patch(landing)
    axis.set_xlim(0, mission.arena_size[0])
    axis.set_ylim(0, mission.arena_size[1])
    axis.set_aspect("equal")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.12)


def _draw_scene_3d(axis, scenario: Scenario) -> None:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    for obstacle in scenario.obstacles:
        color = _obstacle_color(obstacle.kind)
        alpha = 0.10 if obstacle.kind == "boundary" else 0.24 if obstacle.kind == "corridor_wall" else 0.36
        if isinstance(obstacle, RectObstacle):
            axis.bar3d(
                obstacle.x_min,
                obstacle.y_min,
                0.0,
                obstacle.x_max - obstacle.x_min,
                obstacle.y_max - obstacle.y_min,
                obstacle.height,
                color=color,
                alpha=alpha,
                shade=True,
                edgecolor=color,
                linewidth=0.25,
            )
        else:
            theta = np.linspace(0.0, 2.0 * np.pi, 22)
            theta_grid, z_grid = np.meshgrid(theta, np.array([0.0, obstacle.height]))
            x_grid = obstacle.x + obstacle.radius * np.cos(theta_grid)
            y_grid = obstacle.y + obstacle.radius * np.sin(theta_grid)
            axis.plot_surface(
                x_grid,
                y_grid,
                z_grid,
                color=color,
                alpha=alpha,
                linewidth=0.0,
                shade=True,
            )
            top = [
                (obstacle.x + obstacle.radius * np.cos(angle), obstacle.y + obstacle.radius * np.sin(angle), obstacle.height)
                for angle in theta
            ]
            axis.add_collection3d(Poly3DCollection([top], facecolor=color, alpha=alpha, edgecolor=color))

    for zone in scenario.mission.delivery_zones:
        half = zone.half_size
        vertices = [[
            (zone.x - half, zone.y - half, 0.02),
            (zone.x + half, zone.y - half, 0.02),
            (zone.x + half, zone.y + half, 0.02),
            (zone.x - half, zone.y + half, 0.02),
        ]]
        color = DELIVERY_COLORS[zone.label]
        axis.add_collection3d(
            Poly3DCollection(vertices, facecolor=color, edgecolor="#111827", alpha=0.88, linewidth=0.8)
        )
        axis.text(zone.x, zone.y, 0.08, zone.label, color=color, fontsize=8, ha="center")

    mission = scenario.mission
    axis.scatter(
        [mission.start.x],
        [mission.start.y],
        [mission.start.z],
        marker="^",
        s=75,
        color="#f59e0b",
        label="start",
    )
    angles = np.linspace(0.0, 2.0 * np.pi, 80)
    axis.plot(
        mission.landing.x + mission.landing_radius * np.cos(angles),
        mission.landing.y + mission.landing_radius * np.sin(angles),
        np.full_like(angles, 0.025),
        color="#16a34a",
        lw=2,
        label="landing zone",
    )
    axis.set_xlim(0, mission.arena_size[0])
    axis.set_ylim(0, mission.arena_size[1])
    axis.set_zlim(0, mission.arena_height)
    axis.set_box_aspect((mission.arena_size[0], mission.arena_size[1], mission.arena_height))
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.view_init(elev=28, azim=-58)
    axis.grid(alpha=0.18)


def _title(result: SimulationResult) -> str:
    return (
        f"seed={result.metrics.seed}  variant={result.metrics.variant}  "
        f"score={result.metrics.raw_score:.1f}/100  completed={result.metrics.completed}"
    )


def save_run_plot(
    result: SimulationResult,
    output_path: str | Path,
    *,
    show: bool = False,
    view: str = "3d",
) -> None:
    import matplotlib.pyplot as plt

    view = _validate_view(view)
    if view == "3d":
        figure = plt.figure(figsize=(9, 8), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")
        _draw_scene_3d(axis, result.scenario)
    else:
        figure, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
        _draw_scene_2d(axis, result.scenario)

    path = np.asarray(result.metrics.path, dtype=float)
    if len(path):
        if view == "3d":
            axis.plot(path[:, 1], path[:, 2], path[:, 3], color="#e11d48", lw=1.8, label="flight path")
            axis.scatter([path[-1, 1]], [path[-1, 2]], [path[-1, 3]], color="#e11d48", s=35)
        else:
            axis.plot(path[:, 1], path[:, 2], color="#e11d48", lw=1.7, label="flight path")
            axis.scatter(path[-1, 1], path[-1, 2], color="#e11d48", s=30, zorder=5)
    for label, x, y, score in result.metrics.drops:
        if view == "3d":
            axis.scatter([x], [y], [0.06], marker="x", s=55, color="#111827")
            axis.text(x, y, 0.12, f"{label}: {score:g}", fontsize=7)
        else:
            axis.scatter(x, y, marker="x", s=70, color="#111827", zorder=6)
            axis.text(x + 0.08, y - 0.16, f"{label}: {score:g}", fontsize=7)
    axis.legend(loc="upper right", fontsize=8)
    axis.set_title(_title(result))
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    if show:
        plt.show()
    plt.close(figure)


def _path_frames(result: SimulationResult) -> np.ndarray:
    path = np.asarray(result.metrics.path, dtype=float)
    if not len(path):
        start = result.scenario.mission.start
        return np.asarray([[0.0, start.x, start.y, start.z]], dtype=float)
    stride = max(1, int(round(0.5 / 0.1)))
    return path[::stride]


def animate_run(
    result: SimulationResult,
    *,
    save_path: str | Path | None = None,
    view: str = "3d",
) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    view = _validate_view(view)
    frames = _path_frames(result)
    if view == "3d":
        figure = plt.figure(figsize=(9, 8), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")
        _draw_scene_3d(axis, result.scenario)
        line, = axis.plot([], [], [], color="#e11d48", lw=1.9)
        drone = axis.scatter([frames[0, 1]], [frames[0, 2]], [frames[0, 3]], marker="^", color="#e11d48", s=75)
        status = axis.text2D(0.02, 0.97, "", transform=axis.transAxes, va="top")

        def update(frame_index: int):
            current = frames[: frame_index + 1]
            line.set_data(current[:, 1], current[:, 2])
            line.set_3d_properties(current[:, 3])
            drone._offsets3d = ([current[-1, 1]], [current[-1, 2]], [current[-1, 3]])
            status.set_text(
                f"t={current[-1, 0]:.1f}s  z={current[-1, 3]:.2f}m  "
                f"score={result.metrics.raw_score:.1f}/100"
            )
            return line, drone, status

        blit = False
    else:
        figure, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
        _draw_scene_2d(axis, result.scenario)
        line, = axis.plot([], [], color="#e11d48", lw=1.8)
        drone = axis.scatter([], [], marker="^", color="#e11d48", s=70, zorder=7)
        status = axis.text(0.02, 0.98, "", transform=axis.transAxes, va="top")

        def update(frame_index: int):
            current = frames[: frame_index + 1]
            line.set_data(current[:, 1], current[:, 2])
            drone.set_offsets([[current[-1, 1], current[-1, 2]]])
            status.set_text(
                f"t={current[-1, 0]:.1f}s  z={current[-1, 3]:.2f}m  "
                f"score={result.metrics.raw_score:.1f}/100"
            )
            return line, drone, status

        blit = True

    axis.set_title(_title(result))
    animation = FuncAnimation(figure, update, frames=len(frames), interval=35, blit=blit)
    if save_path:
        output = Path(save_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        animation.save(output, writer="pillow", fps=20)
    else:
        plt.show()
    plt.close(figure)
