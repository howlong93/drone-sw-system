#!/usr/bin/env python3
"""Create one machine-readable aggregate for a MIGHTY benchmark run."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median


PLANNER_METRICS = (
    "total_ms", "housekeeping_ms", "global_stage_ms", "global_planner_core_ms",
    "local_stage_ms", "append_plan_ms", "initial_guess_ms", "optimizer_ms",
    "sc_total_ms", "sc_input_prepare_ms", "sc_primary_ms", "sc_secondary_ms",
    "sc_validation_ms", "trajectory_validation_ms", "sc_segments",
    "sc_primary_obstacles", "sc_secondary_obstacles", "sc_primary_hyperplanes",
    "sc_secondary_hyperplanes", "trajectory_min_corridor_margin",
    "sc_max_obstacle_penetration_m",
    "planned_obstacle_violations", "planned_min_occupied_center_distance_m",
    "planned_length_m", "planned_duration_s", "planned_max_velocity",
    "planned_max_acceleration", "planned_max_jerk",
)


def number(value):
    try:
        parsed = float(str(value).strip().rstrip("%"))
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


def distribution(values):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    p95_index = max(0, math.ceil(0.95 * len(values)) - 1)
    return {
        "count": len(values), "min": values[0], "mean": mean(values),
        "median": median(values), "p95": values[p95_index], "max": values[-1],
    }


def read_rows(path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    planner_rows = read_rows(args.run_dir / f"{args.run_id}_planner.csv")
    summary_rows = read_rows(args.run_dir / f"{args.run_id}_planner_summary.csv")
    resource_rows = read_rows(args.run_dir / f"{args.run_id}_resources.csv")
    gpu_rows = read_rows(args.run_dir / f"{args.run_id}_gpu_resources.csv")

    statuses = {}
    for row in planner_rows:
        status = row.get("status", "unknown")
        statuses[status] = statuses.get(status, 0) + 1

    output = {
        "run_id": args.run_id,
        "replan_rows": len(planner_rows),
        "status_counts": statuses,
        "mission": {row["metric"]: number(row["value"]) for row in summary_rows},
        "replan_distributions": {},
        "successful_replan_distributions": {},
        "resources": {},
    }
    for metric in PLANNER_METRICS:
        stats = distribution(number(row.get(metric)) for row in planner_rows)
        if stats is not None:
            output["replan_distributions"][metric] = stats
        successful_stats = distribution(
            number(row.get(metric)) for row in planner_rows if row.get("success") == "1"
        )
        if successful_stats is not None:
            output["successful_replan_distributions"][metric] = successful_stats

    cpu_stats = distribution(number(row.get("cpu_percent")) for row in resource_rows)
    memory_stats = distribution(number(row.get("memory_percent")) for row in resource_rows)
    gpu_stats = distribution(number(row.get("gpu_utilization_percent")) for row in gpu_rows)
    gpu_memory_stats = distribution(number(row.get("gpu_memory_used_mib")) for row in gpu_rows)
    output["resources"] = {
        "container_samples": len(resource_rows), "cpu_percent": cpu_stats,
        "memory_percent": memory_stats, "gpu_samples": len(gpu_rows),
        "host_gpu_utilization_percent": gpu_stats,
        "host_gpu_memory_used_mib": gpu_memory_stats,
    }

    output_path = args.run_dir / f"{args.run_id}_statistics.json"
    output_path.write_text(json.dumps(output, indent=2, sort_keys=True), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
