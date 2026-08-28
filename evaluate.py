from __future__ import annotations

import argparse
import json
from pathlib import Path

from uav_exam.evaluation import evaluate_controller, load_cases, save_evaluation_plot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch-evaluate a UAV exam controller")
    parser.add_argument("--controller", default="student", help="alias or module.path:ClassName")
    parser.add_argument("--cases", default="cases/public_cases.json")
    parser.add_argument(
        "--profiles",
        nargs="+",
        default=["baseline", "conservative", "adaptive"],
    )
    parser.add_argument("--limit", type=int, help="run only the first N cases for a quick check")
    parser.add_argument("--output", default="results/evaluation.json")
    parser.add_argument("--plot", default="results/comparison.png")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cases = load_cases(args.cases)
    if args.limit:
        cases = cases[: args.limit]
    evaluation = evaluate_controller(args.controller, args.profiles, cases)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evaluation, ensure_ascii=False, indent=2), encoding="utf-8")
    save_evaluation_plot(evaluation, args.plot)
    summaries = {
        name: data["summary"] for name, data in evaluation["profiles"].items()
    }
    print(json.dumps(summaries, ensure_ascii=False, indent=2))
    print(f"Detailed results: {output}")
    print(f"Comparison plot: {args.plot}")


if __name__ == "__main__":
    main()
