#!/usr/bin/env python3
"""Validate a recorded UAV path against static Gazebo SDF collision geometry."""

from __future__ import annotations

import argparse
import csv
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Cylinder:
    name: str
    x: float
    y: float
    z: float
    radius: float
    length: float


@dataclass(frozen=True)
class ClearanceSample:
    segment_index: int
    elapsed: float
    x: float
    y: float
    z: float
    clearance: float
    collision: bool
    nearest_obstacle: str


def pose_xyz(element: ET.Element | None) -> tuple[float, float, float]:
    if element is None or not (element.text or "").strip():
        return 0.0, 0.0, 0.0
    values = [float(value) for value in element.text.split()]
    values += [0.0] * (3 - len(values))
    return values[0], values[1], values[2]


def load_cylinders(world_path: Path) -> list[Cylinder]:
    root = ET.parse(world_path).getroot()
    cylinders: list[Cylinder] = []
    for model in root.findall(".//world/model"):
        if (model.findtext("static") or "0").strip() not in {"1", "true", "True"}:
            continue
        mx, my, mz = pose_xyz(model.find("pose"))
        for link in model.findall("link"):
            lx, ly, lz = pose_xyz(link.find("pose"))
            for collision in link.findall("collision"):
                cylinder = collision.find("geometry/cylinder")
                if cylinder is None:
                    continue
                cx, cy, cz = pose_xyz(collision.find("pose"))
                cylinders.append(
                    Cylinder(
                        name=f"{model.get('name', 'model')}::{collision.get('name', 'collision')}",
                        x=mx + lx + cx,
                        y=my + ly + cy,
                        z=mz + lz + cz,
                        radius=float(cylinder.findtext("radius", "0")),
                        length=float(cylinder.findtext("length", "0")),
                    )
                )
    return cylinders


def load_drone_half_extents(config_path: Path) -> tuple[float, float, float]:
    text = config_path.read_text(encoding="utf-8")
    match = re.search(r"^\s*drone_bbox\s*:\s*\[([^]]+)\]", text, re.MULTILINE)
    if not match:
        raise ValueError(f"drone_bbox not found in {config_path}")
    values = [float(value.strip()) for value in match.group(1).split(",")]
    if len(values) != 3:
        raise ValueError("drone_bbox must contain exactly three values")
    return values[0] / 2.0, values[1] / 2.0, values[2] / 2.0


def load_trajectory(path: Path) -> list[tuple[float, float, float, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return [
            (
                float(row["elapsed_from_flight_start_s"]),
                float(row["x_m"]),
                float(row["y_m"]),
                float(row["z_m"]),
            )
            for row in reader
        ]


def interpolate_path(
    points: list[tuple[float, float, float, float]], spacing: float
):
    if not points:
        return
    yield 0, points[0]
    for segment_index, (start, end) in enumerate(zip(points, points[1:]), start=1):
        distance = math.dist(start[1:], end[1:])
        count = max(1, math.ceil(distance / spacing))
        for index in range(1, count + 1):
            alpha = index / count
            yield segment_index, tuple(
                start[axis] + alpha * (end[axis] - start[axis]) for axis in range(4)
            )


def cylinder_clearance(
    x: float,
    y: float,
    z: float,
    cylinder: Cylinder,
    horizontal_radius: float,
    vertical_radius: float,
) -> float:
    radial = math.hypot(x - cylinder.x, y - cylinder.y)
    radial_clearance = radial - (cylinder.radius + horizontal_radius)
    lower = cylinder.z - cylinder.length / 2.0 - vertical_radius
    upper = cylinder.z + cylinder.length / 2.0 + vertical_radius
    if z < lower:
        vertical_clearance = lower - z
    elif z > upper:
        vertical_clearance = z - upper
    else:
        vertical_clearance = -min(z - lower, upper - z)
    if radial_clearance <= 0.0 and vertical_clearance <= 0.0:
        return max(radial_clearance, vertical_clearance)
    return math.hypot(max(0.0, radial_clearance), max(0.0, vertical_clearance))


def update_mission_summary(path: Path, metrics: dict[str, object]) -> None:
    rows: list[tuple[str, str]] = []
    if path.exists():
        with path.open(newline="", encoding="utf-8") as stream:
            rows = [(row["metric"], row["value"]) for row in csv.DictReader(stream)]
    replacements = {key: str(value) for key, value in metrics.items()}
    updated: list[tuple[str, str]] = []
    seen: set[str] = set()
    for key, value in rows:
        updated.append((key, replacements.get(key, value)))
        seen.add(key)
    updated.extend((key, value) for key, value in replacements.items() if key not in seen)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "value"))
        writer.writerows(updated)


def write_clearance_samples(path: Path, samples: list[ClearanceSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "segment_index",
                "elapsed_from_flight_start_s",
                "x_m",
                "y_m",
                "z_m",
                "surface_clearance_m",
                "collision",
                "nearest_obstacle",
            )
        )
        for sample in samples:
            writer.writerow(
                (
                    sample.segment_index,
                    sample.elapsed,
                    sample.x,
                    sample.y,
                    sample.z,
                    sample.clearance,
                    int(sample.collision),
                    sample.nearest_obstacle,
                )
            )


def plot_ground_truth_path(
    path: Path,
    trajectory: list[tuple[float, float, float, float]],
    cylinders: list[Cylinder],
    collision_segments: set[int],
    horizontal_radius: float,
    goal: tuple[float, float, float],
    metrics: dict[str, object],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection, PatchCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(24, 5), dpi=160)
    fig.patch.set_facecolor("#080808")
    axis.set_facecolor("#080808")

    inflated = [
        Circle((item.x, item.y), item.radius + horizontal_radius)
        for item in cylinders
    ]
    physical = [Circle((item.x, item.y), item.radius) for item in cylinders]
    axis.add_collection(
        PatchCollection(inflated, facecolor="#747474", edgecolor="none", alpha=0.28)
    )
    axis.add_collection(
        PatchCollection(
            physical, facecolor="#E8E8E8", edgecolor="#FFFFFF", linewidth=0.25, alpha=0.9
        )
    )

    if len(trajectory) >= 2:
        points = [(row[1], row[2]) for row in trajectory]
        segments = [[points[index - 1], points[index]] for index in range(1, len(points))]
        colors = [
            "#FF2D2D" if index in collision_segments else "#00D5FF"
            for index in range(1, len(points))
        ]
        axis.add_collection(LineCollection(segments, colors=colors, linewidths=2.0, zorder=5))

    if trajectory:
        axis.scatter(
            trajectory[0][1], trajectory[0][2], s=85, c="#20E070", edgecolors="white",
            linewidths=0.5, zorder=7
        )
    axis.scatter(
        goal[0], goal[1], s=95, c="#2166F3", edgecolors="white", linewidths=0.5, zorder=7
    )

    all_x = [item.x for item in cylinders] + [row[1] for row in trajectory] + [goal[0]]
    all_y = [item.y for item in cylinders] + [row[2] for row in trajectory] + [goal[1]]
    if all_x and all_y:
        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)
        axis.set_xlim(xmin - 2.0, xmax + 2.0)
        axis.set_ylim(ymin - 2.0, ymax + 2.0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)", color="white")
    axis.set_ylabel("y (m)", color="white")
    axis.tick_params(colors="white", labelsize=8)
    for spine in axis.spines.values():
        spine.set_color("#777777")
    axis.grid(color="#333333", linewidth=0.35, alpha=0.45)

    safe = int(metrics["ground_truth_collision_free"])
    duration = trajectory[-1][0] if trajectory else math.nan
    axis.set_title(
        "Executed trajectory over Gazebo ground truth  |  "
        f"collision_free={safe}  duration={duration:.1f}s  "
        f"min_clearance={float(metrics['ground_truth_min_surface_clearance_m']):.3f}m",
        color="white",
        fontsize=11,
    )
    legend = [
        Line2D([0], [0], color="#00D5FF", lw=2, label="collision-free trajectory"),
        Line2D([0], [0], color="#FF2D2D", lw=2, label="collision segment"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#20E070", label="start"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#2166F3", label="goal"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#E8E8E8", label="obstacle"),
    ]
    axis.legend(handles=legend, loc="upper center", ncol=5, frameon=False, labelcolor="white")
    fig.tight_layout()
    fig.savefig(path, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mission-summary", type=Path)
    parser.add_argument("--plot-output", type=Path)
    parser.add_argument("--clearance-output", type=Path)
    parser.add_argument("--goal-x", type=float, default=305.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=3.0)
    parser.add_argument("--sample-spacing", type=float, default=0.05)
    args = parser.parse_args()

    trajectory = load_trajectory(args.trajectory)
    cylinders = load_cylinders(args.world)
    half_x, half_y, half_z = load_drone_half_extents(args.config)
    horizontal_radius = max(half_x, half_y)

    collision_samples = 0
    collision_segments: set[int] = set()
    validation_samples = 0
    min_clearance = math.inf
    first_collision_time = math.nan
    first_collision_obstacle = ""
    clearance_samples: list[ClearanceSample] = []
    for segment_index, point in interpolate_path(trajectory, args.sample_spacing):
        elapsed, x, y, z = point
        validation_samples += 1
        nearest_name = ""
        nearest_clearance = math.inf
        for cylinder in cylinders:
            clearance = cylinder_clearance(
                x, y, z, cylinder, horizontal_radius, half_z
            )
            if clearance < nearest_clearance:
                nearest_clearance = clearance
                nearest_name = cylinder.name
        min_clearance = min(min_clearance, nearest_clearance)
        collision = nearest_clearance <= 0.0
        clearance_samples.append(
            ClearanceSample(
                segment_index,
                elapsed,
                x,
                y,
                z,
                nearest_clearance,
                collision,
                nearest_name,
            )
        )
        if collision:
            collision_samples += 1
            collision_segments.add(segment_index)
            if math.isnan(first_collision_time):
                first_collision_time = elapsed
                first_collision_obstacle = nearest_name

    collision_free = bool(trajectory) and collision_samples == 0
    metrics: dict[str, object] = {
        "ground_truth_validation_success": int(bool(trajectory) and bool(cylinders)),
        "ground_truth_collision_free": int(collision_free),
        "executed_trajectory_safe": int(collision_free),
        "ground_truth_collision_samples": collision_samples,
        "ground_truth_collision_segments": len(collision_segments),
        "ground_truth_min_surface_clearance_m": min_clearance,
        "ground_truth_first_collision_time_s": first_collision_time,
        "ground_truth_first_collision_obstacle": first_collision_obstacle,
        "trajectory_samples_recorded": len(trajectory),
        "ground_truth_validation_samples": validation_samples,
        "ground_truth_obstacle_count": len(cylinders),
        "ground_truth_sample_spacing_m": args.sample_spacing,
        "drone_half_extent_x_m": half_x,
        "drone_half_extent_y_m": half_y,
        "drone_half_extent_z_m": half_z,
        "ground_truth_world_file": args.world,
        "ground_truth_path_plot": args.plot_output or "",
        "ground_truth_clearance_file": args.clearance_output or "",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "value"))
        writer.writerows(metrics.items())
    if args.mission_summary:
        update_mission_summary(args.mission_summary, metrics)
    if args.clearance_output:
        write_clearance_samples(args.clearance_output, clearance_samples)
    if args.plot_output:
        plot_ground_truth_path(
            args.plot_output,
            trajectory,
            cylinders,
            collision_segments,
            horizontal_radius,
            (args.goal_x, args.goal_y, args.goal_z),
            metrics,
        )
    print(
        f"[GROUND TRUTH] collision_free={int(collision_free)} "
        f"collision_samples={collision_samples} min_clearance_m={min_clearance:.6f}"
    )
    return 0 if trajectory and cylinders else 2


if __name__ == "__main__":
    raise SystemExit(main())
