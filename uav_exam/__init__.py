"""Public API for the UAV examination simulator.

``EgoLikePlanner`` is an educational abstraction with the same *role* as a
local quadrotor planner.  It is not the original EGO-Planner implementation.
"""

from .planner import EgoLikePlanner
from .simulator import Simulator, run_episode
from .types import (
    Action,
    ActionMode,
    ControllerProfile,
    DeliveryZone,
    GridMap,
    MISSION_MAX_SCORE,
    MissionSpec,
    Observation,
    PlanResult,
    PlannerParams,
    PlanStatus,
    Pose,
    TargetDetection,
    TrajectoryPoint,
)

__all__ = [
    "Action",
    "ActionMode",
    "ControllerProfile",
    "DeliveryZone",
    "EgoLikePlanner",
    "GridMap",
    "MISSION_MAX_SCORE",
    "MissionSpec",
    "Observation",
    "PlanResult",
    "PlannerParams",
    "PlanStatus",
    "Pose",
    "Simulator",
    "TargetDetection",
    "TrajectoryPoint",
    "run_episode",
]
