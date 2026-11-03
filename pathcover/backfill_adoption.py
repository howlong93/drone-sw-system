#!/usr/bin/env python3
import bisect
import csv
import json
import os
import sys

import numpy as np
import rosbag


run_dir = os.path.abspath(sys.argv[1])
bag_path = next(
    os.path.join(run_dir, name) for name in os.listdir(run_dir) if name.endswith(".bag")
)

goal_stamp = None
corridors = {}
all_poly_stamps = []
with rosbag.Bag(bag_path) as bag:
    for topic, msg, _ in bag.read_messages(topics=["/move_base_simple/goal", "/quadrotor/polytopes"]):
        if topic == "/move_base_simple/goal":
            stamp = msg.header.stamp.to_sec()
            goal_stamp = stamp if goal_stamp is None else min(goal_stamp, stamp)
            continue
        key = (int(msg.header.stamp.secs), int(msg.header.stamp.nsecs))
        corridor = []
        for poly in msg.polytope:
            if len(poly.b) and len(poly.A) == 3 * len(poly.b):
                corridor.append((
                    np.asarray(poly.A, dtype=np.float64).reshape((-1, 3)),
                    np.asarray(poly.b, dtype=np.float64),
                ))
        corridors[key] = corridor
        all_poly_stamps.append((msg.header.stamp.to_sec(), key))

post_goal = [(stamp, key) for stamp, key in all_poly_stamps if goal_stamp is not None and stamp >= goal_stamp]
source_indices = {key: index for index, (_, key) in enumerate(post_goal)}

adoption_path = os.path.join(run_dir, "corridor_adoption.csv")
with open(adoption_path, newline="") as handle:
    adoption_rows = list(csv.DictReader(handle))

resolved = []
for row in adoption_rows:
    source_stamp = float(row["source_corridor_stamp_s"])
    sec = int(source_stamp)
    nsec = int(round((source_stamp - sec) * 1.0e9))
    if nsec >= 1000000000:
        sec += 1
        nsec -= 1000000000
    key = (sec, nsec)
    corridor = corridors.get(key)
    if corridor is None:
        raise RuntimeError("Missing source corridor in bag: %r" % (key,))
    start_index = int(row["start_polytope_index"])
    adopted = corridor[start_index:]
    row["source_message_index"] = source_indices.get(key, "")
    row["adopted_polytope_count"] = len(adopted)
    row["matched_source_corridor"] = "True"
    resolved.append((float(row["adoption_sim_time_s"]), int(row["adoption_index"]), source_stamp, adopted))

temp_path = adoption_path + ".tmp"
with open(temp_path, "w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(adoption_rows[0].keys()))
    writer.writeheader()
    writer.writerows(adoption_rows)
os.replace(temp_path, adoption_path)

resolved.sort()
adoption_times = [item[0] for item in resolved]
trajectory_path = os.path.join(run_dir, "trajectory_quality.csv")
with open(trajectory_path, newline="") as handle:
    trajectory_rows = list(csv.DictReader(handle))

containment = []
violations = []
for row in trajectory_rows:
    sim_time = float(row["sim_time_s"])
    event_pos = bisect.bisect_right(adoption_times, sim_time) - 1
    if event_pos < 0:
        row["inside_adopted_corridor"] = ""
        row["corridor_union_violation_m"] = ""
        row["active_corridor_source_stamp_s"] = ""
        row["active_adoption_index"] = ""
        continue
    _, event_index, source_stamp, corridor = resolved[event_pos]
    point = np.asarray([float(row["x"]), float(row["y"]), float(row["z"])])
    poly_violations = []
    for a_matrix, b_vector in corridor:
        norms = np.linalg.norm(a_matrix, axis=1)
        valid = norms > 1.0e-12
        if np.any(valid):
            poly_violations.append(float(np.max(
                (a_matrix[valid].dot(point) - b_vector[valid]) / norms[valid]
            )))
    union_violation = min(poly_violations) if poly_violations else None
    inside = union_violation is not None and union_violation <= 1.0e-6
    row["inside_adopted_corridor"] = str(inside) if union_violation is not None else ""
    row["corridor_union_violation_m"] = union_violation if union_violation is not None else ""
    row["active_corridor_source_stamp_s"] = source_stamp
    row["active_adoption_index"] = event_index
    if union_violation is not None:
        containment.append(inside)
        violations.append(union_violation)

temp_path = trajectory_path + ".tmp"
with open(temp_path, "w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(trajectory_rows[0].keys()))
    writer.writeheader()
    writer.writerows(trajectory_rows)
os.replace(temp_path, trajectory_path)

summary_path = os.path.join(run_dir, "summary.json")
with open(summary_path) as handle:
    summary = json.load(handle)
summary["corridor_adoption"]["matched_event_count"] = len(adoption_rows)
summary["corridor_adoption"]["postprocessed_from_rosbag"] = True
summary["corridor_quality"]["raw_corridor_message_count_in_rosbag_after_goal"] = len(post_goal)
summary["trajectory_quality"]["containment_sample_count"] = len(containment)
summary["trajectory_quality"]["corridor_containment_rate"] = (
    sum(containment) / len(containment) if containment else None
)
summary["trajectory_quality"]["maximum_corridor_union_violation_m"] = (
    max(violations) if violations else None
)
summary["quality_method"]["corridor_adoption_postprocessing"] = (
    "All adoption events and trajectory containment were re-associated offline from the complete rosbag, "
    "so live subscriber queue drops do not remove source-corridor information."
)
temp_path = summary_path + ".tmp"
with open(temp_path, "w") as handle:
    json.dump(summary, handle, indent=2, sort_keys=True)
os.replace(temp_path, summary_path)

print("adoptions", len(adoption_rows))
print("matched", len(adoption_rows))
print("post_goal_corridors", len(post_goal))
print("trajectory_rows", len(trajectory_rows))
print("containment_samples", len(containment))
print("containment_rate", summary["trajectory_quality"]["corridor_containment_rate"])
print("max_violation_m", summary["trajectory_quality"]["maximum_corridor_union_violation_m"])
