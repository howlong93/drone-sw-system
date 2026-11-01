# PathCover Quadrotor system benchmark

All simulation and GPU work runs on the 4060 Ti server. MobaXterm only provides the SSH terminal and X11 display.

## Run once

Connect with MobaXterm X11 forwarding, then:

```bash
cd ~/PathCover
./run_benchmark_drone.sh
```

The default case starts at the upper-left corner `(-18.5, 8.5)` and flies to the lower-left corner of the lower-right room `(9.0, -8.5)`. The runner waits for odometry, the voxel map, and the planner's goal subscriber before publishing the goal. It ends the complete launch automatically after the vehicle remains within `0.60 m` of the goal for two seconds, or after a 240-second timeout.

If X11 is unavailable, the same command automatically runs headless and still produces all benchmark artifacts.

## Output

Each run creates a timestamped directory:

```text
benchmark_results/start_-18.5_8.5__goal_9.0_-8.5/
  quadrotor_left_top_to_right_room_left_bottom_YYYYMMDD_HHMMSS/
    summary.json
    pathcover_timing.csv
    remaining_points.csv
    corridor_quality.csv
    polytope_quality.csv
    overlap_quality.csv
    trajectory_quality.csv
    voxel_map_final.npz
    quadrotor_left_top_to_right_room_left_bottom_YYYYMMDD_HHMMSS.bag
```

`summary.json` contains success/timeout status, duration, traveled distance, final error,
PathCover timing, corridor geometry, trajectory containment, and obstacle-clearance summaries.
The quality CSV files preserve deterministic periodic geometry measurements (every 40th corridor
message by default), while the bag retains every corridor for offline recomputation. The compressed NPZ
contains the final sampled voxel map, and the bag keeps every corridor plus a throttled voxel-map
stream for later analysis without the size of the full-rate map topic.

## Override a case

```bash
START_X=-18.5 START_Y=8.5 GOAL_X=9.0 GOAL_Y=-8.5 TIMEOUT_S=300 \
  ./run_benchmark_drone.sh my_run_001
```

Run IDs may contain only letters, digits, `_`, `-`, and `.`. A pre-existing run directory is never overwritten.

## Stop early

Press `Ctrl+C`. The runner attempts to close the rosbag and writes an `interrupted` summary before the disposable container exits.
