# Gazebo forest collision investigation

## Scope

This note records the collision behaviour observed in the 30-run Gazebo forest batch
`mighty_baseline_20261118_105956`, the decisions that must remain unchanged, and the
current investigation/fix order.

## Observed results

- All 30 runs completed and produced a ground-truth summary, clearance CSV, and path overlay.
- Mission outcome: 24 reached the goal and 6 ended in flight timeout (80% mission success).
- Ground-truth safety outcome: 0/30 collision-free.
- Successful runs averaged 249.77 s flight time and 322.83 m executed path length.
- Successful runs encountered 161 local-goal-occupied events; all 161 retries succeeded and
  no relocation failure was recorded.
- The ground-truth validator found an average of 2264.9 collision samples per run.
- The average minimum surface clearance was -1.481 m (range: -1.632 to -0.965 m).
- The first collision occurred after 5.10 s on average (range: 3.88 to 5.40 s).
- All 30 runs first collided with the same object, `136::collision`, located at approximately
  `(7.025, 1.019, 3.0) m`, with radius 1.013 m and cylinder length 6 m.

The validator uses the Gazebo cylinder collision geometry and half of
`drone_bbox=[0.5, 0.5, 0.5]`, sampled at 0.05 m path spacing. It does not add
`inflation_hgp`, so the recorded penetrations are not caused by double-counting the planner
inflation margin.

## Current evidence and working diagnosis

### 1. Point-cloud-to-map TF failures

Across the 30 console logs, approximately 4151 point-cloud callbacks reported:

```text
[PointCloudCallback] lookupTransform failed:
"map" passed to lookupTransform argument target_frame does not exist
```

The warning appeared in every run. When this transform is unavailable, a LiDAR point cloud
cannot be transformed from the sensor frame into the `map` frame, so that cloud cannot update
the occupied voxel map. The deterministic first collision with obstacle 136 strongly suggests
that the mission can start before the first local map is ready.

Every run also logged a missing `occupancy_grid.min_occupied_neighbors` parameter. This must
be corrected or explicitly verified as harmless after checking the mapper's effective default.

### 2. Unknown-space policy

Unknown space must remain traversable. The desired behaviour is that a previously unknown
obstacle is sensed early enough, inserted into the occupied map, and avoided by a subsequent
replan.

Current configuration deliberately keeps:

```yaml
sfc_use_unknown_as_obstacle: false
use_soft_cost_obstacles: false
planner_Co: 0.0
```

The secondary corridor attempt treats unknown as occupied, but the 30 logs contain 1369
`dilate failed` messages. In the current implementation, secondary-corridor failure falls back
to the whole corridor. No primary convex-decomposition failure was found in this batch.

### 3. Planning-map versus Gazebo-ground-truth mismatch

The most likely immediate cause is not that the planner knowingly accepts an occupied voxel,
but that the occupied voxel is missing or late in the online planning map. Fixing the
point-cloud/TF readiness problem should therefore be verified before changing corridor or
optimizer parameters.

RISP cannot avoid an obstacle that is absent from its input obstacle map, so this issue must be
resolved before evaluating RISP integration.

### 4. Gazebo contact response

Forest cylinders currently contain:

```xml
<collide_without_contact>1</collide_without_contact>
```

This allows the simulated vehicle to continue through collision geometry instead of being
physically stopped by contact response. Keep this behaviour for the current software-planning
stage so post-run collision analysis remains possible without aborting the mission.

For a later physics-realistic simulation/demo, disable this option and add/enable a Gazebo
contact sensor so a physical collision is independently recorded and prevents pass-through.

### 5. Mission-success semantics

Keep the current definition: `mission_success` means the vehicle reached the terminal goal.
Collision remains an independent post-run safety result and must not currently stop trajectory
publication or terminate the mission.

## Fix and verification order

1. Compare the original `~/mighty` launch/TF setup with `~/drone-sw-system`.
2. Correct the missing or late `map -> NX01/NX01_livox` transform path.
3. Do not send the first mission goal until TF and the occupied map are demonstrably ready.
4. Add low-overhead readiness evidence: successful/dropped point-cloud counters, map-update age,
   and occupied-voxel count at mission start.
5. Correct or explicitly configure `occupancy_grid.min_occupied_neighbors`.
6. In a single-run verification, confirm that obstacle 136 appears in the mapper output and the
   HGP planning map before the vehicle reaches it.
7. Re-run a small batch. If the obstacle exists in the planning map but the executed path still
   intersects it, then inspect HGP path generation, corridor construction, trajectory
   optimization, and controller tracking separately.

## Known reporting issue

The batch summary contains eight rows with `timeout_scope=flight`, but two of them (run 020 and
run 027) also recorded `mission_success=1`. This is likely a timeout/reached-marker race and
should be fixed separately from the collision investigation.
