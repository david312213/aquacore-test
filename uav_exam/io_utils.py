from __future__ import annotations

import importlib
import json
from pathlib import Path
from typing import Any

from .types import ControllerProfile


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def load_profile(name_or_path: str) -> ControllerProfile:
    path = Path(name_or_path)
    if not path.exists():
        path = PROJECT_ROOT / "configs" / f"{name_or_path}.json"
    if not path.exists():
        raise FileNotFoundError(f"controller profile not found: {name_or_path}")
    with path.open("r", encoding="utf-8") as stream:
        return ControllerProfile.from_dict(json.load(stream))


def load_controller(spec: str, profile: ControllerProfile) -> Any:
    aliases = {
        "student": "student_controller:StudentController",
        "baseline": "uav_exam.baseline_controller:BaselineController",
    }
    module_name, separator, class_name = aliases.get(spec, spec).partition(":")
    if not separator:
        raise ValueError("controller must be an alias or use module.path:ClassName")
    module = importlib.import_module(module_name)
    controller_class = getattr(module, class_name)
    try:
        return controller_class(profile=profile)
    except TypeError as exc:
        raise TypeError(
            f"{class_name} must accept a keyword argument named profile"
        ) from exc
