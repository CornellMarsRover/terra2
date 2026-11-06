#!/usr/bin/env python3
"""Run the closed-loop astronaut simulation without ROS or hardware."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "src" / "astronaut_autonomy"))

from astronaut_autonomy.competition_simulation import (  # noqa: E402
    CompetitionSimulationConfig,
    run_competition_simulation,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/astronaut_sim")
    parser.add_argument("--range-noise", type=float, default=0.0)
    parser.add_argument("--bearing-noise", type=float, default=0.0)
    parser.add_argument(
        "--command-input-mode",
        choices=("gesture", "voice", "matching", "conflict"),
        default="gesture",
        help="route commands through the selected fusion scenario",
    )
    parser.add_argument("--no-failure-injection", action="store_true")
    args = parser.parse_args()

    output = Path(args.output)
    config = CompetitionSimulationConfig(
        range_noise_std_m=args.range_noise,
        bearing_noise_std_deg=args.bearing_noise,
        inject_failures=not args.no_failure_injection,
        command_input_mode=args.command_input_mode,
    )
    report = run_competition_simulation(
        config,
        csv_path=output / "trajectory.csv",
        report_path=output / "report.json",
    )
    print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    return 0 if report.succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
