from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Protocol


class Obstacle(Protocol):
    height: float
    kind: str

    def signed_distance(self, x: float, y: float) -> float: ...

    def contains(self, x: float, y: float) -> bool: ...


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float
    height: float
    kind: str

    def signed_distance(self, x: float, y: float) -> float:
        return hypot(x - self.x, y - self.y) - self.radius

    def contains(self, x: float, y: float) -> bool:
        return self.signed_distance(x, y) <= 0.0


@dataclass(frozen=True)
class RectObstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    height: float
    kind: str

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)

    def signed_distance(self, x: float, y: float) -> float:
        dx = max(self.x_min - x, 0.0, x - self.x_max)
        dy = max(self.y_min - y, 0.0, y - self.y_max)
        outside = hypot(dx, dy)
        if dx > 0.0 or dy > 0.0:
            return outside
        return -min(x - self.x_min, self.x_max - x, y - self.y_min, self.y_max - y)

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def disk_collides(obstacle: Obstacle, x: float, y: float, radius: float) -> bool:
    return obstacle.signed_distance(x, y) <= radius


def point_clear(
    obstacles: list[Obstacle], x: float, y: float, margin: float, *, ignore_kinds: set[str] | None = None
) -> bool:
    ignored = ignore_kinds or set()
    return all(obs.kind in ignored or obs.signed_distance(x, y) > margin for obs in obstacles)
