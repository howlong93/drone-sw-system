#!/usr/bin/env python3
import rosbag
import importlib.util
import time
import numpy as np

path = "benchmark_results/start_-18.5_8.5__goal_9.0_-8.5/corridor_adoption_validation_20261014_03/corridor_adoption_validation_20261014_03.bag"
with rosbag.Bag(path) as bag:
    rows = []
    for _, msg, stamp in bag.read_messages(topics=["/quadrotor/polytopes"]):
        if stamp.to_sec() >= 18.0:
            rows.append((stamp.to_sec(), msg.header.stamp.to_sec(), len(msg.polytope), [len(p.b) for p in msg.polytope]))
        if len(rows) == 45:
            break
for index, row in enumerate(rows):
    if index < 5 or index in (39, 40, 41, 44):
        print(index, row)

spec = importlib.util.spec_from_file_location(
    "runner", "src/corridor_planning/scripts/pathcover_benchmark_runner.py"
)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
with rosbag.Bag(path) as bag:
    for _, msg, stamp in bag.read_messages(topics=["/quadrotor/polytopes"]):
        if stamp.to_sec() < 18.0:
            continue
        started = time.monotonic()
        polytopes = []
        for poly in msg.polytope:
            a = np.asarray(poly.A, dtype=np.float64).reshape((-1, 3))
            b = np.asarray(poly.b, dtype=np.float64)
            seed = np.asarray(poly.seed[:3], dtype=np.float64)
            polytopes.append((a, b, seed, runner.describe_polytope(a, b, seed=seed)))
        for i in range(len(polytopes) - 1):
            runner.estimate_overlap(polytopes[i], polytopes[i + 1], i)
        print("first_message_geometry_s", time.monotonic() - started)
        break
