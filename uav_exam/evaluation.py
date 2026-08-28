from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .io_utils import load_controller, load_profile
from .simulator import run_episode
from .types import MISSION_MAX_SCORE


@dataclass(frozen=True)
class EvaluationCase:
    seed: int
    variant: str


def load_cases(path: str | Path) -> list[EvaluationCase]:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = json.load(stream)
    return [EvaluationCase(int(item["seed"]), str(item["variant"])) for item in raw]


def evaluate_controller(
    controller_spec: str,
    profile_names: list[str],
    cases: list[EvaluationCase],
) -> dict[str, Any]:
    output: dict[str, Any] = {"controller": controller_spec, "profiles": {}}
    for profile_name in profile_names:
        profile = load_profile(profile_name)
        runs: list[dict[str, Any]] = []
        for case in cases:
            controller = load_controller(controller_spec, profile)
            result = run_episode(
                controller,
                seed=case.seed,
                variant=case.variant,
                profile=profile,
            )
            runs.append(result.metrics.to_dict())
        output["profiles"][profile.name] = {
            "summary": aggregate_runs(runs),
            "runs": runs,
        }
    return output


def aggregate_runs(runs: list[dict[str, Any]]) -> dict[str, Any]:
    if not runs:
        raise ValueError("at least one run is required")
    all_drop_errors = [
        float(error)
        for run in runs
        for error in run.get("drop_errors", {}).values()
    ]
    return {
        "run_count": len(runs),
        "average_score": round(float(np.mean([run["raw_score"] for run in runs])), 3),
        "score_std": round(float(np.std([run["raw_score"] for run in runs])), 3),
        "completion_rate": round(float(np.mean([run["completed"] for run in runs])), 4),
        "collision_rate": round(float(np.mean([not run["collision_free"] for run in runs])), 4),
        "fatal_collision_rate": round(float(np.mean([run["fatal_collision"] for run in runs])), 4),
        "two_door_rate": round(float(np.mean([run["door_passes"] == 2 for run in runs])), 4),
        "mean_drop_error": round(float(np.mean(all_drop_errors)), 4) if all_drop_errors else None,
        "mean_duration": round(float(np.mean([run["duration"] for run in runs])), 3),
        "mean_controller_ms": round(float(np.mean([run["mean_controller_ms"] for run in runs])), 4),
        "mean_planner_ms": round(float(np.mean([run["mean_planner_ms"] for run in runs])), 4),
        "mean_replan_compliance": round(
            float(np.mean([run["replan_compliance"] for run in runs])), 4
        ),
        "safety_violations": int(sum(run["safety_violations"] for run in runs)),
    }


def save_evaluation_plot(evaluation: dict[str, Any], output_path: str | Path) -> None:
    import matplotlib.pyplot as plt

    profile_items = list(evaluation["profiles"].items())
    names = [name for name, _ in profile_items]
    summaries = [data["summary"] for _, data in profile_items]
    figure, axes = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
    charts = (
        ("average_score", "Average mission score", (0, MISSION_MAX_SCORE)),
        ("completion_rate", "Completion rate", (0, 1)),
        ("collision_rate", "Collision rate (lower is better)", (0, 1)),
        ("mean_duration", "Mean simulated time (s)", None),
    )
    colors = ["#64748b", "#0ea5e9", "#22c55e"]
    for axis, (key, title, limits) in zip(axes.flat, charts):
        values = [summary[key] for summary in summaries]
        bars = axis.bar(names, values, color=colors[: len(names)])
        axis.set_title(title)
        if limits:
            axis.set_ylim(*limits)
        axis.grid(axis="y", alpha=0.2)
        axis.bar_label(bars, fmt="%.2f", padding=2)
    figure.suptitle(f"Controller evaluation: {evaluation['controller']}")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
