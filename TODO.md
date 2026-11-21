# Development TODO

## Safety gate and recovery

`gate_v1.2.0` is an experimental safety-layer checkpoint. The gate can reject
candidate trajectories and truncate a committed plan when they intersect the
current inflated planning map, but it is not yet suitable as the default mode
for mission-completion benchmarks.

- [ ] Replace the recovery trigger based only on consecutive candidate
  rejections. In the 2026-11-21 ten-run batch, all five timed-out runs reached
  only 6--11 consecutive rejections (threshold: 15), so recovery never ran.
  Combine plan-buffer time remaining, lack of vehicle progress, hover duration,
  and repeated rejection activity into an explicit deadlock detector.
- [ ] Prevent rejection-counter resets caused by intermittent planner outcomes
  from hiding a persistent deadlock. Record the reset reason and preserve a
  rolling-window rejection/stall score.
- [ ] Verify and tune stop-state replanning. Confirm that its initial position,
  zero velocity/acceleration state, map snapshot, and local-goal occupancy
  checks are consistent with the normal planner path.
- [ ] Improve alternate local-goal selection. Reject occupied or unreachable
  variants before optimization, account for corridor connectivity, and record
  why each variant fails.
- [ ] Validate the retreat fallback end to end. The latest ten-run batch
  recorded zero retreats, so activation, history selection, current-map
  revalidation, command publication, and return to normal replanning still need
  targeted tests.
- [ ] Define bounded recovery escalation: stop-state replan, alternate goal,
  validated retreat, then a controlled terminal hover/failure state. Never
  publish a trajectory that fails a hard structural or occupied-voxel check.
- [ ] Separate gate guarantees from ground-truth safety. The gate sees only the
  current inflated planning map; obstacles observed late or absent from that map
  can still cause Gazebo ground-truth collisions. Report both known-map gate
  safety and world-ground-truth collision results.
- [ ] Improve obstacle observation/map update latency and continue committed-plan
  revalidation as new occupied voxels arrive. Measure sensor-to-map and
  map-to-revalidation latency before attributing collisions to the optimizer.
- [ ] Add deterministic gated/ungated A/B benchmarks with identical seeds,
  worlds, goals, and timeouts. Compare mission success, ground-truth collision
  rate/severity, flight time, hover time, rejection count, truncations, recovery
  entries, recovery success, and timeout rate.
- [ ] Revisit gate defaults only after recovery no longer causes a major mission
  success regression. For PathCover safe-corridor integration experiments,
  keep the gate configurable and run with `ENABLE_SAFETY_GATE=0` when isolating
  corridor-generation behavior.

