from __future__ import annotations

import argparse
import json
from pathlib import Path

from uav_exam.io_utils import load_controller, load_profile
from uav_exam.simulator import run_episode
from uav_exam.visualize import animate_run, save_run_plot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one UAV examination scenario")
    parser.add_argument("--controller", default="student", help="alias or module.path:ClassName")
    parser.add_argument("--profile", default="adaptive", help="profile name or JSON path")
    parser.add_argument("--seed", type=int, default=1001)
    parser.add_argument(
        "--variant",
        default="normal",
        choices=(
            "normal",
            "tight_door",
            "delivery_near_obstacle",
            "target_near_obstacle",
            "odom_dropout",
            "map_dropout",
        ),
    )
    parser.add_argument(
        "--view",
        choices=("3d", "2d"),
        default="3d",
        help="spatial visualization mode (default: 3d)",
    )
    parser.add_argument("--json", dest="json_path", help="write detailed metrics JSON")
    parser.add_argument("--plot", help="write a final path PNG")
    parser.add_argument("--animate", action="store_true", help="open an interactive replay")
    parser.add_argument("--gif", help="save a replay GIF (uses Pillow via Matplotlib)")
    parser.add_argument("--events", action="store_true", help="print event log")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    profile = load_profile(args.profile)
    controller = load_controller(args.controller, profile)
    result = run_episode(
        controller,
        seed=args.seed,
        variant=args.variant,
        profile=profile,
    )
    data = result.metrics.to_dict()
    print(json.dumps(data, ensure_ascii=False, indent=2))
    if args.events:
        print("\nEvents:")
        print("\n".join(result.metrics.events) or "(none)")
    if args.json_path:
        output = Path(args.json_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.plot:
        save_run_plot(result, args.plot, view=args.view)
    if args.gif:
        animate_run(result, save_path=args.gif, view=args.view)
    elif args.animate:
        animate_run(result, view=args.view)


if __name__ == "__main__":
    main()
