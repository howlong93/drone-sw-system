#!/usr/bin/env python3
"""Validate the artifacts and summarize every run in one benchmark case."""

import argparse
import csv
import json
import sys
from pathlib import Path


def csv_has_data(path: Path) -> bool:
    with path.open(newline="") as stream:
        rows = csv.reader(stream)
        next(rows, None)
        return next(rows, None) is not None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--expected-runs", type=int)
    parser.add_argument(
        "--check-rosbag",
        action="store_true",
        help="open each bag with ROS and verify that it contains messages",
    )
    args = parser.parse_args()

    rosbag = None
    if args.check_rosbag:
        try:
            import rosbag as rosbag_module
        except ImportError as exc:
            parser.error(f"--check-rosbag requires a sourced ROS environment: {exc}")
        rosbag = rosbag_module

    run_dirs = sorted(path for path in args.case_dir.iterdir() if path.is_dir())
    problems = []
    if args.expected_runs is not None and len(run_dirs) != args.expected_runs:
        problems.append(
            f"expected {args.expected_runs} run directories, found {len(run_dirs)}"
        )

    print("run\tstatus\tsim_s\tpath_m\tfinal_error_m\tartifacts")
    for run_dir in run_dirs:
        summary_path = run_dir / "summary.json"
        if not summary_path.is_file():
            problems.append(f"{run_dir.name}: missing summary.json")
            continue

        try:
            summary = json.loads(summary_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            problems.append(f"{run_dir.name}: invalid summary.json: {exc}")
            continue

        artifacts = summary.get("artifacts", {})
        missing = []
        for artifact_name in artifacts.values():
            artifact_path = run_dir / artifact_name
            if not artifact_path.is_file() or artifact_path.stat().st_size == 0:
                missing.append(artifact_name)
            elif artifact_path.suffix == ".csv" and not csv_has_data(artifact_path):
                missing.append(f"{artifact_name} (no data rows)")

        if rosbag is not None and artifacts.get("rosbag") not in missing:
            bag_name = artifacts.get("rosbag")
            if bag_name:
                try:
                    with rosbag.Bag(str(run_dir / bag_name), "r") as bag:
                        if bag.get_message_count() == 0:
                            missing.append(f"{bag_name} (no messages)")
                except Exception as exc:  # rosbag exposes several format exceptions
                    missing.append(f"{bag_name} (unreadable: {exc})")

        if len(artifacts) != 9:
            problems.append(
                f"{run_dir.name}: expected 9 declared artifacts, found {len(artifacts)}"
            )
        if missing:
            problems.append(f"{run_dir.name}: incomplete artifacts: {', '.join(missing)}")

        status = summary.get("status", "missing")
        if status != "success" or summary.get("success") is not True:
            problems.append(f"{run_dir.name}: simulation status is {status!r}")

        print(
            f"{run_dir.name}\t{status}\t{summary.get('sim_duration_s')}\t"
            f"{summary.get('path_length_m')}\t{summary.get('final_error_m')}\t"
            f"{len(artifacts) - len(missing)}/{len(artifacts)}"
        )

    if problems:
        print("\nValidation problems:", file=sys.stderr)
        for problem in problems:
            print(f"- {problem}", file=sys.stderr)
        return 1

    print(f"\nValidation passed: {len(run_dirs)} complete successful runs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
