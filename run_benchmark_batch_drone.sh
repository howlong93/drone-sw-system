#!/usr/bin/env bash

set -uo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_DIR}"

COUNT="${1:-20}"
WORLD="${ENV:-hard_forest}"
TIMEOUT_S="${TIMEOUT_S:-240}"
AUTO_EXIT_DELAY="${AUTO_EXIT_DELAY:-3}"
INTER_RUN_DELAY="${INTER_RUN_DELAY:-2}"
GPU="${GPU:-1}"
MIGHTY_IMAGE="${MIGHTY_IMAGE:-mighty:latest}"
BATCH_ID="${BATCH_ID:-$(date +%Y%m%d_%H%M%S)}"
STATISTICS_DIR="${STATISTICS_DIR:-${REPO_DIR}/statistics}"

[[ "${COUNT}" =~ ^[1-9][0-9]*$ ]] || { echo "[ERROR] count must be a positive integer." >&2; exit 2; }
[[ "${TIMEOUT_S}" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]] || { echo "[ERROR] TIMEOUT_S must be positive." >&2; exit 2; }
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

if ! command -v script >/dev/null 2>&1 || ! command -v timeout >/dev/null 2>&1; then
  echo "[ERROR] This runner requires util-linux 'script' and GNU 'timeout'." >&2
  exit 1
fi

mkdir -p "${STATISTICS_DIR}"
STATISTICS_DIR="$(cd -- "${STATISTICS_DIR}" && pwd)"
BATCH_DIR="${STATISTICS_DIR}/${BATCH_ID}"
mkdir -p "${BATCH_DIR}"
SUMMARY_FILE="${BATCH_DIR}/${BATCH_ID}_batch_summary.csv"
printf 'run_index,run_id,started_at,ended_at,duration_s,exit_code,result,mission_success,planner_rows,run_directory\n' > "${SUMMARY_FILE}"

echo "Batch configuration:"
echo "  Runs            : ${COUNT}"
echo "  World           : ${WORLD}"
echo "  Timeout/run     : ${TIMEOUT_S} second(s)"
echo "  Auto-exit delay : ${AUTO_EXIT_DELAY} second(s)"
echo "  Output          : ${BATCH_DIR}"

failures=0
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
    "BENCHMARK=1"
    "BENCHMARK_RUN_ID=${run_id}"
    "BENCHMARK_RUN_DIR=${run_dir}"
    "CONTAINER_NAME=${container_name}"
    "ENV=${WORLD}"
    "AUTO_EXIT_DELAY=${AUTO_EXIT_DELAY}"
    "GPU=${GPU}"
    "MIGHTY_IMAGE=${MIGHTY_IMAGE}"
  )
  for coordinate_name in GOAL_X GOAL_Y GOAL_Z; do
    if [[ -n "${!coordinate_name:-}" ]]; then
      command_parts+=("${coordinate_name}=${!coordinate_name}")
    fi
  done
  command_parts+=("${REPO_DIR}/run_gazebo_forest.sh" 0 0)
  printf -v run_command '%q ' "${command_parts[@]}"

  resource_file="${run_dir}/${run_id}_resources.csv"
  gpu_resource_file="${run_dir}/${run_id}_gpu_resources.csv"
  printf 'timestamp,cpu_percent,memory_usage,memory_percent,pids,network_io,block_io\n' > "${resource_file}"
  printf 'timestamp,gpu_utilization_percent,gpu_memory_used_mib\n' > "${gpu_resource_file}"

  timeout --signal=INT --kill-after=30 "${TIMEOUT_S}" \
    script --quiet --return --command "${run_command}" "${run_dir}/${run_id}_console.log" &
  runner_pid=$!

  while kill -0 "${runner_pid}" >/dev/null 2>&1; do
    sample_time="$(date --iso-8601=seconds)"
    if docker container inspect "${container_name}" >/dev/null 2>&1; then
      docker stats --no-stream \
        --format "${sample_time},{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.PIDs}},{{.NetIO}},{{.BlockIO}}" \
        "${container_name}" >> "${resource_file}" 2>/dev/null || true
    fi
    if [[ "${GPU}" == "1" ]] && command -v nvidia-smi >/dev/null 2>&1; then
      gpu_sample="$(nvidia-smi --query-gpu=utilization.gpu,memory.used \
        --format=csv,noheader,nounits 2>/dev/null | head -n 1 | tr -d ' ')"
      [[ -n "${gpu_sample}" ]] && printf '%s,%s\n' "${sample_time}" "${gpu_sample}" >> "${gpu_resource_file}"
    fi
    sleep 1
  done
  wait "${runner_pid}"
  status=$?

  if docker container inspect "${container_name}" >/dev/null 2>&1; then
    echo "[WARN] Removing leftover container ${container_name}." >&2
    docker rm --force "${container_name}" >/dev/null
  fi

  ended_at="$(date --iso-8601=seconds)"
  ended_epoch="$(date +%s)"
  duration_s=$((ended_epoch - started_epoch))
  planner_file="${run_dir}/${run_id}_planner.csv"
  planner_rows=0
  if [[ -s "${planner_file}" ]]; then
    planner_rows=$(( $(wc -l < "${planner_file}") - 1 ))
    (( planner_rows < 0 )) && planner_rows=0
  fi

  python3 "${REPO_DIR}/scripts/summarize_benchmark.py" \
    --run-dir "${run_dir}" --run-id "${run_id}" || \
    echo "[WARN] Could not generate aggregate statistics for ${run_id}." >&2

  mission_success="0"
  planner_summary_file="${run_dir}/${run_id}_planner_summary.csv"
  if [[ -s "${planner_summary_file}" ]]; then
    mission_success="$(awk -F, '$1 == "mission_success" { print $2; exit }' "${planner_summary_file}")"
  fi

  result="success"
  if (( status != 0 )) || (( planner_rows == 0 )) || [[ "${mission_success}" != "1" ]]; then
    result="failed"
    failures=$((failures + 1))
    echo "===== Failed ${run_id}: exit=${status}, planner_rows=${planner_rows}, mission_success=${mission_success} =====" >&2
  else
    echo "===== Completed ${run_id}: planner_rows=${planner_rows} ====="
  fi

  printf '%d,%s,%s,%s,%d,%d,%s,%s,%d,%s\n' \
    "${index}" "${run_id}" "${started_at}" "${ended_at}" "${duration_s}" \
    "${status}" "${result}" "${mission_success}" "${planner_rows}" "${run_dir}" >> "${SUMMARY_FILE}"

  if (( index < COUNT )) && [[ "${INTER_RUN_DELAY}" != "0" ]]; then
    sleep "${INTER_RUN_DELAY}"
  fi
done

printf '\nBatch finished: %d requested, %d failed.\n' "${COUNT}" "${failures}"
echo "Summary: ${SUMMARY_FILE}"
(( failures == 0 ))
