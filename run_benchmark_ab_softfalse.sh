#!/usr/bin/env bash

set -uo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_DIR}"

RUNS_PER_MODE="${RUNS_PER_MODE:-10}"
TIMEOUT_S="${TIMEOUT_S:-300}"
AUTO_EXIT_DELAY="${AUTO_EXIT_DELAY:-3}"
INTER_RUN_DELAY="${INTER_RUN_DELAY:-2}"
EXPERIMENT_ID="${EXPERIMENT_ID:-benchmark_ab_softfalse_$(date +%Y%m%d_%H%M%S)}"
ROOT_DIR="${REPO_DIR}/statistics/${EXPERIMENT_ID}"
SUMMARY_FILE="${ROOT_DIR}/${EXPERIMENT_ID}_comparison.csv"

[[ "${RUNS_PER_MODE}" =~ ^[1-9][0-9]*$ ]] || { echo "[ERROR] RUNS_PER_MODE must be positive." >&2; exit 2; }
[[ "${TIMEOUT_S}" =~ ^[1-9][0-9]*$ ]] || { echo "[ERROR] TIMEOUT_S must be positive." >&2; exit 2; }

# The comparison is only meaningful in hard-obstacle mode. Check both the
# working tree and the image actually used by run_gazebo_forest.sh.
host_soft="$(awk '/use_soft_cost_obstacles:/ {print $2; exit}' config/mighty.yaml)"
image_soft="$(docker run --rm --entrypoint awk mighty:latest \
  '/use_soft_cost_obstacles:/ {print $2; exit}' \
  /home/kkondo/code/mighty_ws/install/mighty/share/mighty/config/mighty.yaml)"
if [[ "${host_soft}" != "false" || "${image_soft}" != "false" ]]; then
  echo "[ERROR] soft-obstacle mode mismatch: host=${host_soft}, image=${image_soft}" >&2
  exit 2
fi

mkdir -p "${ROOT_DIR}"
printf 'sequence,mode,arm_index,run_id,started_at,ended_at,duration_s,exit_code,reached_goal,planner_rows,run_directory\n' > "${SUMMARY_FILE}"

declare -A arm_index=( [on]=0 [off]=0 )
sequence=0

for ((pair = 1; pair <= RUNS_PER_MODE; ++pair)); do
  # Alternate within-pair order to reduce warm-up/time-order bias.
  if (( pair % 2 == 1 )); then
    modes=(on off)
  else
    modes=(off on)
  fi

  for mode in "${modes[@]}"; do
    sequence=$((sequence + 1))
    arm_index[${mode}]=$((arm_index[${mode}] + 1))
    index="${arm_index[${mode}]}"
    printf -v padded_index '%03d' "${index}"
    run_id="${EXPERIMENT_ID}_${mode}_run_${padded_index}"
    run_dir="${ROOT_DIR}/${run_id}"
    container_name="mighty-gazebo-forest-${EXPERIMENT_ID}-${mode}-${padded_index}"
    mkdir -p "${run_dir}"

    benchmark_flag=0
    [[ "${mode}" == "on" ]] && benchmark_flag=1
    started_at="$(date --iso-8601=seconds)"
    started_epoch="$(date +%s)"

    command_parts=(
      env
      "BENCHMARK=${benchmark_flag}"
      "BENCHMARK_RUN_ID=${run_id}"
      "BENCHMARK_RUN_DIR=${run_dir}"
      "CONTAINER_NAME=${container_name}"
      "AUTO_EXIT_DELAY=${AUTO_EXIT_DELAY}"
      "MIGHTY_IMAGE=mighty:latest"
      "${REPO_DIR}/run_gazebo_forest.sh" 0 0
    )
    printf -v run_command '%q ' "${command_parts[@]}"

    echo "===== A/B ${sequence}/$((RUNS_PER_MODE * 2)): ${mode} ${index}/${RUNS_PER_MODE} ====="
    timeout --signal=INT --kill-after=30 "${TIMEOUT_S}" \
      script --quiet --return --command "${run_command}" "${run_dir}/${run_id}_console.log"
    status=$?

    if docker container inspect "${container_name}" >/dev/null 2>&1; then
      docker rm --force "${container_name}" >/dev/null
    fi

    ended_at="$(date --iso-8601=seconds)"
    ended_epoch="$(date +%s)"
    duration_s=$((ended_epoch - started_epoch))
    reached_goal=0
    planner_rows=0

    if [[ "${mode}" == "on" ]]; then
      planner_file="${run_dir}/${run_id}_planner.csv"
      summary_file="${run_dir}/${run_id}_planner_summary.csv"
      if [[ -s "${planner_file}" ]]; then
        planner_rows=$(( $(wc -l < "${planner_file}") - 1 ))
      fi
      if [[ -s "${summary_file}" ]]; then
        reached_goal="$(awk -F, '$1 == "mission_success" {print $2; exit}' "${summary_file}")"
      fi
    elif (( status == 0 && duration_s > 20 )); then
      # With AUTO_EXIT_DELAY enabled and no benchmark file, a normal return is
      # produced by the /NX01/goal_reached monitor killing the tmux session.
      reached_goal=1
    fi

    printf '%d,%s,%d,%s,%s,%s,%d,%d,%s,%d,%s\n' \
      "${sequence}" "${mode}" "${index}" "${run_id}" "${started_at}" "${ended_at}" \
      "${duration_s}" "${status}" "${reached_goal}" "${planner_rows}" "${run_dir}" >> "${SUMMARY_FILE}"

    echo "===== Result: mode=${mode}, reached=${reached_goal}, exit=${status}, duration=${duration_s}s ====="
    sleep "${INTER_RUN_DELAY}"
  done
done

echo "A/B comparison complete: ${SUMMARY_FILE}"
