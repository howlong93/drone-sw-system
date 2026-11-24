#!/usr/bin/env bash

set -uo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_DIR}"

COUNT="${1:-30}"
RVIZ_GUI="${BATCH_RVIZ_GUI:-0}"
GAZEBO_GUI="${BATCH_GAZEBO_GUI:-0}"
RECORD_STATISTICS="${BATCH_RECORD_STATISTICS:-1}"
BENCHMARK_REPLAN_DETAILS="${BATCH_BENCHMARK_REPLAN_DETAILS:-0}"
ENABLE_SAFETY_GATE="${BATCH_ENABLE_SAFETY_GATE:-1}"
SC_METHOD="${BATCH_SC_METHOD:-pathcover_line}"
RISP_ALPHA="${BATCH_RISP_ALPHA:-0.05}"
SC_ELLIPSOID_FALLBACK="${BATCH_SC_ELLIPSOID_FALLBACK:-0}"
RISP_RANDOM_SEED="${BATCH_RISP_RANDOM_SEED:-1}"
PATHCOVER_MAX_CORRIDORS="${BATCH_PATHCOVER_MAX_CORRIDORS:-12}"
PATHCOVER_OVERLAP_DISTANCE="${BATCH_PATHCOVER_OVERLAP_DISTANCE:-0.25}"
PATHCOVER_MIN_OVERLAP_MARGIN="${BATCH_PATHCOVER_MIN_OVERLAP_MARGIN:-0.001}"
WORLD="${ENV:-hard_forest}"
TIMEOUT_S="${TIMEOUT_S:-330}"
FLIGHT_TIMEOUT_S="${FLIGHT_TIMEOUT_S:-270}"
AUTO_EXIT_DELAY="${AUTO_EXIT_DELAY:-3}"
INTER_RUN_DELAY="${INTER_RUN_DELAY:-2}"
GPU="${GPU:-1}"
MIGHTY_IMAGE="${MIGHTY_IMAGE:-mighty:latest}"
BATCH_ID="${BATCH_ID:-$(date +%Y%m%d_%H%M%S)}"
STATISTICS_DIR="${STATISTICS_DIR:-${REPO_DIR}/statistics}"

[[ "${COUNT}" =~ ^[1-9][0-9]*$ ]] || { echo "[ERROR] count must be a positive integer." >&2; exit 2; }
[[ "${RVIZ_GUI}" == "0" || "${RVIZ_GUI}" == "1" ]] || { echo "[ERROR] BATCH_RVIZ_GUI must be 0 or 1." >&2; exit 2; }
[[ "${GAZEBO_GUI}" == "0" || "${GAZEBO_GUI}" == "1" ]] || { echo "[ERROR] BATCH_GAZEBO_GUI must be 0 or 1." >&2; exit 2; }
[[ "${RECORD_STATISTICS}" == "0" || "${RECORD_STATISTICS}" == "1" ]] || { echo "[ERROR] BATCH_RECORD_STATISTICS must be 0 or 1." >&2; exit 2; }
[[ "${BENCHMARK_REPLAN_DETAILS}" == "0" || "${BENCHMARK_REPLAN_DETAILS}" == "1" ]] || { echo "[ERROR] BATCH_BENCHMARK_REPLAN_DETAILS must be 0 or 1." >&2; exit 2; }
[[ "${ENABLE_SAFETY_GATE}" == "0" || "${ENABLE_SAFETY_GATE}" == "1" ]] || { echo "[ERROR] BATCH_ENABLE_SAFETY_GATE must be 0 or 1." >&2; exit 2; }
[[ "${SC_METHOD}" == "ellipsoid" || "${SC_METHOD}" == "risp_point" || "${SC_METHOD}" == "risp_line" || "${SC_METHOD}" == "pathcover_line" ]] || { echo "[ERROR] BATCH_SC_METHOD is invalid." >&2; exit 2; }
awk -v value="${RISP_ALPHA}" 'BEGIN { exit !(value > 0 && value < 1) }' || { echo "[ERROR] BATCH_RISP_ALPHA must be in (0, 1)." >&2; exit 2; }
[[ "${SC_ELLIPSOID_FALLBACK}" == "0" || "${SC_ELLIPSOID_FALLBACK}" == "1" ]] || { echo "[ERROR] BATCH_SC_ELLIPSOID_FALLBACK must be 0 or 1." >&2; exit 2; }
[[ "${RISP_RANDOM_SEED}" =~ ^[0-9]+$ ]] || { echo "[ERROR] BATCH_RISP_RANDOM_SEED must be non-negative." >&2; exit 2; }
[[ "${PATHCOVER_MAX_CORRIDORS}" =~ ^([2-9]|[1-9][0-9]+)$ ]] || { echo "[ERROR] BATCH_PATHCOVER_MAX_CORRIDORS must be at least 2." >&2; exit 2; }
awk -v value="${PATHCOVER_OVERLAP_DISTANCE}" 'BEGIN { exit !(value >= 0) }' || { echo "[ERROR] BATCH_PATHCOVER_OVERLAP_DISTANCE must be non-negative." >&2; exit 2; }
awk -v value="${PATHCOVER_MIN_OVERLAP_MARGIN}" 'BEGIN { exit !(value >= 0) }' || { echo "[ERROR] BATCH_PATHCOVER_MIN_OVERLAP_MARGIN must be non-negative." >&2; exit 2; }
[[ "${TIMEOUT_S}" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "[ERROR] TIMEOUT_S must be positive." >&2; exit 2; }
[[ "${FLIGHT_TIMEOUT_S}" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "[ERROR] FLIGHT_TIMEOUT_S must be positive." >&2; exit 2; }
[[ "${AUTO_EXIT_DELAY}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "[ERROR] AUTO_EXIT_DELAY must be non-negative for batch runs." >&2; exit 2; }
[[ "${INTER_RUN_DELAY}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "[ERROR] INTER_RUN_DELAY must be non-negative." >&2; exit 2; }
[[ "${GPU}" == "0" || "${GPU}" == "1" ]] || { echo "[ERROR] GPU must be 0 or 1." >&2; exit 2; }
[[ "${BATCH_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "[ERROR] BATCH_ID contains unsupported characters." >&2; exit 2; }

for coordinate_name in GOAL_X GOAL_Y GOAL_Z; do
  coordinate_value="${!coordinate_name:-}"
  if [[ -n "${coordinate_value}" && ! "${coordinate_value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] ${coordinate_name} must be numeric (received: ${coordinate_value})." >&2
    exit 2
  fi
done

if ! command -v timeout >/dev/null 2>&1; then
  echo "[ERROR] This runner requires GNU 'timeout'." >&2
  exit 1
fi

mkdir -p "${STATISTICS_DIR}"
STATISTICS_DIR="$(cd -- "${STATISTICS_DIR}" && pwd)"
BATCH_DIR="${STATISTICS_DIR}/${BATCH_ID}"
mkdir -p "${BATCH_DIR}"
SUMMARY_FILE="${BATCH_DIR}/${BATCH_ID}_batch_summary.csv"
printf 'run_index,run_id,started_at,ended_at,duration_s,exit_code,result,mission_result,safety_result,timeout_scope,mission_success,planner_rows,run_directory\n' > "${SUMMARY_FILE}"

echo "Batch configuration:"
echo "  Runs            : ${COUNT}"
echo "  RViz GUI        : ${RVIZ_GUI}"
echo "  Gazebo GUI      : ${GAZEBO_GUI}"
echo "  Statistics      : ${RECORD_STATISTICS}"
echo "  Heavy details   : ${BENCHMARK_REPLAN_DETAILS}"
echo "  Safety gate     : ${ENABLE_SAFETY_GATE}"
echo "  SC method       : ${SC_METHOD}"
echo "  RISP alpha      : ${RISP_ALPHA}"
echo "  SC fallback     : ${SC_ELLIPSOID_FALLBACK}"
echo "  RISP seed       : ${RISP_RANDOM_SEED}"
echo "  PC max corridors: ${PATHCOVER_MAX_CORRIDORS}"
echo "  PC overlap dist : ${PATHCOVER_OVERLAP_DISTANCE}"
echo "  PC min margin   : ${PATHCOVER_MIN_OVERLAP_MARGIN}"
echo "  Folder name     : ${BATCH_ID}"
echo "  World           : ${WORLD}"
echo "  Timeout/run     : ${TIMEOUT_S} second(s)"
echo "  Flight timeout  : ${FLIGHT_TIMEOUT_S} second(s) after first trajectory"
echo "  Auto-exit delay : ${AUTO_EXIT_DELAY} second(s)"
echo "  Output          : ${BATCH_DIR}"

failures=0
safety_failures=0
for ((index = 1; index <= COUNT; index++)); do
  printf -v padded_index '%03d' "${index}"
  run_id="${BATCH_ID}_run_${padded_index}"
  run_dir="${BATCH_DIR}/${run_id}"
  container_name="mighty-gazebo-forest-${BATCH_ID}-${padded_index}"
  mkdir -p "${run_dir}"

  started_at="$(date --iso-8601=seconds)"
  started_epoch="$(date +%s)"
  printf '\n===== Batch run %d/%d: %s =====\n' "${index}" "${COUNT}" "${run_id}"

  command_parts=(
    env
    "BENCHMARK=${RECORD_STATISTICS}"
    "BENCHMARK_REPLAN_DETAILS=${BENCHMARK_REPLAN_DETAILS}"
    "ENABLE_SAFETY_GATE=${ENABLE_SAFETY_GATE}"
    "SC_METHOD=${SC_METHOD}"
    "RISP_ALPHA=${RISP_ALPHA}"
    "SC_ELLIPSOID_FALLBACK=${SC_ELLIPSOID_FALLBACK}"
    "RISP_RANDOM_SEED=${RISP_RANDOM_SEED}"
    "PATHCOVER_MAX_CORRIDORS=${PATHCOVER_MAX_CORRIDORS}"
    "PATHCOVER_OVERLAP_DISTANCE=${PATHCOVER_OVERLAP_DISTANCE}"
    "PATHCOVER_MIN_OVERLAP_MARGIN=${PATHCOVER_MIN_OVERLAP_MARGIN}"
    "BENCHMARK_RUN_ID=${run_id}"
    "BENCHMARK_RUN_DIR=${run_dir}"
    "CONTAINER_NAME=${container_name}"
    "ENV=${WORLD}"
    "AUTO_EXIT_DELAY=${AUTO_EXIT_DELAY}"
    "GPU=${GPU}"
    "MIGHTY_IMAGE=${MIGHTY_IMAGE}"
    "FORCE_TTY=1"
  )
  for coordinate_name in GOAL_X GOAL_Y GOAL_Z; do
    if [[ -n "${!coordinate_name:-}" ]]; then
      command_parts+=("${coordinate_name}=${!coordinate_name}")
    fi
  done
  command_parts+=("${REPO_DIR}/run_gazebo_forest.sh" "${RVIZ_GUI}" "${GAZEBO_GUI}")

  flight_marker="${run_dir}/${run_id}_planner_flight_started.marker"

  # Minimal benchmark mode: avoid a PTY, full console capture, and resource
  # polling during flight. Normal output is discarded; rare stderr output is
  # retained for diagnosis. Flight metrics remain in memory until finalization.
  timeout --signal=INT --kill-after=30 "${TIMEOUT_S}" \
    "${command_parts[@]}" >/dev/null 2>"${run_dir}/${run_id}_errors.log" &
  runner_pid=$!

  flight_started_epoch=""
  flight_timeout=0
  while kill -0 "${runner_pid}" >/dev/null 2>&1; do
    now_epoch="$(date +%s)"
    if [[ -z "${flight_started_epoch}" && -s "${flight_marker}" ]]; then
      flight_started_epoch="${now_epoch}"
      echo "[BATCH] Flight timer started for ${run_id}."
    fi
    if [[ -n "${flight_started_epoch}" ]] &&
       awk -v now="${now_epoch}" -v start="${flight_started_epoch}" -v limit="${FLIGHT_TIMEOUT_S}" \
         'BEGIN { exit !((now - start) >= limit) }'; then
      flight_timeout=1
      echo "[WARN] Flight timeout reached for ${run_id}; finalizing benchmark data." >&2

      # Stop the planner node first, rather than stopping Docker's PID 1
      # immediately. MIGHTY writes the in-memory executed trajectory and
      # summary from its destructor, allowing the host-side ground-truth
      # validator to produce an overlay even for timed-out missions. This is a
      # one-time timeout operation and adds no I/O to the flight loop.
      docker exec "${container_name}" pkill --signal SIGINT --exact mighty \
        >/dev/null 2>&1 || true

      if [[ "${RECORD_STATISTICS}" == "1" ]]; then
        timeout_finalize_complete=0
        for _ in {1..20}; do
          if [[ -s "${run_dir}/${run_id}_planner_trajectory.csv" &&
                -s "${run_dir}/${run_id}_planner_summary.csv" ]]; then
            timeout_finalize_complete=1
            break
          fi
          sleep 1
        done
        if (( timeout_finalize_complete == 0 )); then
          echo "[WARN] Timed-out planner did not finish benchmark finalization within 20 seconds." >&2
        fi
      else
        # Give rclcpp a short grace period even when no statistics are enabled.
        sleep 2
      fi

      echo "[WARN] Benchmark finalization requested; stopping the simulation container." >&2
      docker kill --signal=SIGINT "${container_name}" >/dev/null 2>&1 || true
      kill -INT "${runner_pid}" >/dev/null 2>&1 || true
      break
    fi
    sleep 1
  done
  wait "${runner_pid}" 2>/dev/null
  status=$?

  timeout_scope="none"
  if (( flight_timeout == 1 )); then
    timeout_scope="flight"
    status=124
  elif (( status == 124 )); then
    timeout_scope="outer"
  fi

  if docker container inspect "${container_name}" >/dev/null 2>&1; then
    echo "[WARN] Requesting final graceful shutdown for leftover container ${container_name}." >&2
    docker kill --signal=SIGINT "${container_name}" >/dev/null 2>&1 || true
    for _ in {1..10}; do
      docker container inspect "${container_name}" >/dev/null 2>&1 || break
      sleep 1
    done
    if docker container inspect "${container_name}" >/dev/null 2>&1; then
      echo "[WARN] Force-removing unresponsive container ${container_name}." >&2
      docker rm --force "${container_name}" >/dev/null
    fi
  fi

  ended_at="$(date --iso-8601=seconds)"
  ended_epoch="$(date +%s)"
  duration_s=$((ended_epoch - started_epoch))
  planner_file="${run_dir}/${run_id}_planner.csv"
  planner_summary_file="${run_dir}/${run_id}_planner_summary.csv"
  trajectory_file="${run_dir}/${run_id}_planner_trajectory.csv"
  ground_truth_file="${run_dir}/${run_id}_ground_truth_summary.csv"
  overlay_file="${run_dir}/${run_id}_ground_truth_path.png"
  clearance_file="${run_dir}/${run_id}_trajectory_clearance.csv"
  planner_rows=0
  if [[ -s "${planner_file}" ]]; then
    planner_rows=$(( $(wc -l < "${planner_file}") - 1 ))
    (( planner_rows < 0 )) && planner_rows=0
  fi

  mission_success="0"
  if [[ -s "${planner_summary_file}" ]]; then
    mission_success="$(awk -F, '$1 == "mission_success" { gsub(/\r/, "", $2); print $2; exit }' "${planner_summary_file}")"
  elif [[ "${RECORD_STATISTICS}" == "0" && "${status}" == "0" ]]; then
    # Without planner benchmarking there is no planner summary. A clean
    # auto-exit is the available mission-completion signal.
    mission_success="1"
    printf 'metric,value\nmission_success,1\n' > "${planner_summary_file}"
  else
    printf 'metric,value\nmission_success,0\n' > "${planner_summary_file}"
  fi

  if [[ -s "${trajectory_file}" ]]; then
    goal_args=()
    [[ -n "${GOAL_X:-}" ]] && goal_args+=(--goal-x "${GOAL_X}")
    [[ -n "${GOAL_Y:-}" ]] && goal_args+=(--goal-y "${GOAL_Y}")
    [[ -n "${GOAL_Z:-}" ]] && goal_args+=(--goal-z "${GOAL_Z}")
    python3 "${REPO_DIR}/scripts/validate_trajectory_ground_truth.py" \
      --trajectory "${trajectory_file}" \
      --world "${REPO_DIR}/worlds/${WORLD}.world" \
      --config "${REPO_DIR}/config/mighty.yaml" \
      --output "${ground_truth_file}" \
      --mission-summary "${planner_summary_file}" \
      --plot-output "${overlay_file}" \
      --clearance-output "${clearance_file}" \
      "${goal_args[@]}" || \
      echo "[WARN] Ground-truth validation/plot failed for ${run_id}." >&2
  fi

  if [[ "${RECORD_STATISTICS}" == "1" ]]; then
    python3 "${REPO_DIR}/scripts/summarize_benchmark.py" \
      --run-dir "${run_dir}" --run-id "${run_id}" || \
      echo "[WARN] Could not generate aggregate statistics for ${run_id}." >&2
  fi

  ground_truth_collision_free=""
  if [[ -s "${ground_truth_file}" ]]; then
    ground_truth_collision_free="$(awk -F, '$1 == "ground_truth_collision_free" { gsub(/\r/, "", $2); print $2; exit }' "${ground_truth_file}")"
  fi

  mission_result="mission_failed"
  if [[ "${mission_success}" == "1" ]]; then
    mission_result="reached"
  elif [[ "${timeout_scope}" == "flight" ]]; then
    mission_result="flight_timeout"
  elif [[ "${timeout_scope}" == "outer" ]]; then
    mission_result="outer_timeout"
  elif (( status != 0 )); then
    mission_result="runtime_error"
  fi

  safety_result="not_validated"
  if [[ "${ground_truth_collision_free}" == "1" ]]; then
    safety_result="safe"
  elif [[ "${ground_truth_collision_free}" == "0" ]]; then
    safety_result="collision"
    safety_failures=$((safety_failures + 1))
  fi

  result="success"
  if [[ "${mission_success}" != "1" ]]; then
    result="failed"
    failures=$((failures + 1))
    echo "===== Mission failed ${run_id}: ${mission_result}, exit=${status} =====" >&2
  else
    echo "===== Mission reached ${run_id}: safety=${safety_result}, planner_rows=${planner_rows} ====="
  fi

  printf '%d,%s,%s,%s,%d,%d,%s,%s,%s,%s,%s,%d,%s\n' \
    "${index}" "${run_id}" "${started_at}" "${ended_at}" "${duration_s}" \
    "${status}" "${result}" "${mission_result}" "${safety_result}" "${timeout_scope}" \
    "${mission_success}" "${planner_rows}" "${run_dir}" >> "${SUMMARY_FILE}"

  if (( index < COUNT )) && [[ "${INTER_RUN_DELAY}" != "0" ]]; then
    sleep "${INTER_RUN_DELAY}"
  fi
done

printf '\nBatch finished: %d requested, %d mission failures, %d safety failures.\n' \
  "${COUNT}" "${failures}" "${safety_failures}"
echo "Summary: ${SUMMARY_FILE}"
(( failures == 0 ))
