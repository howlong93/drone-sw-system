#!/usr/bin/env bash

set -Eeuo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# User-facing default: one command launches a reproducible 30-run headless
# benchmark. The batch runner invokes this script with explicit `0 0`, so the
# branch cannot recurse. Set SINGLE_RUN=1 to retain the old no-argument GUI run.
if (( $# == 0 )) && [[ "${SINGLE_RUN:-0}" != "1" ]]; then
  export BATCH_ID="${BATCH_ID:-mighty_baseline_$(date +%Y%m%d_%H%M%S)}"
  exec "${REPO_DIR}/run_benchmark_batch_drone.sh" "${BENCHMARK_COUNT:-30}"
fi

usage() {
  cat <<'EOF'
Usage: ./run_gazebo_forest.sh [RVIZ_GUI] [GAZEBO_GUI]

With no arguments, run 30 headless benchmark trials and archive all results.
With GUI arguments, run one README "Single UAV through a Gazebo forest" trial.

  RVIZ_GUI=1    Show RViz (default: 1).
  RVIZ_GUI=0    Do not start RViz.
  GAZEBO_GUI=1  Show the Gazebo client (default: 0).
  GAZEBO_GUI=0  Run Gazebo server-only.

Examples:
  ./run_gazebo_forest.sh        # 30-run headless benchmark
  SINGLE_RUN=1 ./run_gazebo_forest.sh  # one run: RViz on, Gazebo GUI off
  ./run_gazebo_forest.sh 1 1    # RViz and Gazebo GUI on
  ./run_gazebo_forest.sh 0 1    # Gazebo GUI only
  ./run_gazebo_forest.sh 0 0    # Fully headless

Optional environment variables:
  MIGHTY_IMAGE   Docker image to run (default: mighty:latest)
  ENV            Gazebo world (default: hard_forest)
  GOAL_X/Y/Z     Goal components (default selected by run_sim.py: 305 0 3)
  GPU            Set to 0 to omit Docker GPU access (default: 1)
  DEV            Set to 1 to mount the current source/config/launch/rviz (default: 0)
  AUTO_EXIT_DELAY Seconds to wait after reaching the goal before exiting
                  (default: 3; 0 exits immediately; -1 disables auto-exit)
  BENCHMARK       Set to 1 to save planner timing data (default: 0)
  BENCHMARK_RUN_ID Identifier used for the output folder and CSV filename
  BENCHMARK_RUN_DIR Host output directory; defaults to statistics/<run-id>
  CONTAINER_NAME  Docker container name (default: mighty-gazebo-forest)
  DRY_RUN        Set to 1 to print the Docker command without running it
  BENCHMARK_COUNT Number of trials for no-argument batch mode (default: 30)
  SINGLE_RUN      Set to 1 to make a no-argument invocation run one GUI trial
EOF
}

RVIZ_GUI="${1:-1}"
GAZEBO_GUI="${2:-0}"

if [[ "${RVIZ_GUI}" == "-h" || "${RVIZ_GUI}" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# > 2 )); then
  echo "[ERROR] Expected at most two arguments." >&2
  usage >&2
  exit 2
fi

for variable in RVIZ_GUI GAZEBO_GUI; do
  if [[ "${!variable}" != "0" && "${!variable}" != "1" ]]; then
    echo "[ERROR] ${variable} must be 0 or 1 (received: ${!variable})." >&2
    usage >&2
    exit 2
  fi
done

if [[ "${RVIZ_GUI}" == "1" || "${GAZEBO_GUI}" == "1" ]]; then
  X11_ENABLED=1
else
  X11_ENABLED=0
fi

IMAGE="${MIGHTY_IMAGE:-mighty:latest}"
WORLD="${ENV:-hard_forest}"
GPU="${GPU:-1}"
DEV="${DEV:-0}"
AUTO_EXIT_DELAY="${AUTO_EXIT_DELAY:-3}"
BENCHMARK="${BENCHMARK:-0}"
CONTAINER_NAME="${CONTAINER_NAME:-mighty-gazebo-forest}"

if [[ "${GPU}" != "0" && "${GPU}" != "1" ]]; then
  echo "[ERROR] GPU must be 0 or 1 (received: ${GPU})." >&2
  exit 2
fi

if [[ "${DEV}" != "0" && "${DEV}" != "1" ]]; then
  echo "[ERROR] DEV must be 0 or 1 (received: ${DEV})." >&2
  exit 2
fi

if [[ "${BENCHMARK}" != "0" && "${BENCHMARK}" != "1" ]]; then
  echo "[ERROR] BENCHMARK must be 0 or 1 (received: ${BENCHMARK})." >&2
  exit 2
fi

if [[ ! "${CONTAINER_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "[ERROR] CONTAINER_NAME contains unsupported characters: ${CONTAINER_NAME}" >&2
  exit 2
fi

if [[ ! "${AUTO_EXIT_DELAY}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v value="${AUTO_EXIT_DELAY}" 'BEGIN { exit !(value >= -1 && (value == -1 || value >= 0)) }'; then
  echo "[ERROR] AUTO_EXIT_DELAY must be -1 or a non-negative number (received: ${AUTO_EXIT_DELAY})." >&2
  exit 2
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[ERROR] docker is not available." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[ERROR] Docker image '${IMAGE}' was not found." >&2
  echo "        Build it with: (cd '${REPO_DIR}/docker' && make build-local)" >&2
  exit 1
fi

BENCHMARK_RUN_ID="${BENCHMARK_RUN_ID:-}"
BENCHMARK_RUN_DIR="${BENCHMARK_RUN_DIR:-}"
BENCHMARK_HOST_FILE=""
BENCHMARK_CONTAINER_FILE=""
BENCHMARK_TRAJECTORY_FILE=""
BENCHMARK_SUMMARY_FILE=""
BENCHMARK_GROUND_TRUTH_FILE=""
BENCHMARK_OVERLAY_FILE=""
BENCHMARK_CLEARANCE_FILE=""
if [[ "${BENCHMARK}" == "1" ]]; then
  BENCHMARK_RUN_ID="${BENCHMARK_RUN_ID:-forest_$(date +%Y%m%d_%H%M%S)_run_001}"
  if [[ ! "${BENCHMARK_RUN_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
    echo "[ERROR] BENCHMARK_RUN_ID contains unsupported characters: ${BENCHMARK_RUN_ID}" >&2
    exit 2
  fi
  BENCHMARK_RUN_DIR="${BENCHMARK_RUN_DIR:-${REPO_DIR}/statistics/${BENCHMARK_RUN_ID}}"
  mkdir -p "${BENCHMARK_RUN_DIR}"
  BENCHMARK_RUN_DIR="$(cd -- "${BENCHMARK_RUN_DIR}" && pwd)"
  BENCHMARK_HOST_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_planner.csv"
  BENCHMARK_CONTAINER_FILE="/mighty_statistics/${BENCHMARK_RUN_ID}_planner.csv"
  BENCHMARK_TRAJECTORY_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_planner_trajectory.csv"
  BENCHMARK_SUMMARY_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_planner_summary.csv"
  BENCHMARK_GROUND_TRUTH_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_ground_truth_summary.csv"
  BENCHMARK_OVERLAY_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_ground_truth_path.png"
  BENCHMARK_CLEARANCE_FILE="${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_trajectory_clearance.csv"
fi

docker_args=(
  run
  --rm
  --name "${CONTAINER_NAME}"
  --interactive
  --privileged
  --network=host
  --hostname "$(hostname)"
  --env "MODE=gazebo"
  --env "ENV=${WORLD}"
  --env "USE_RVIZ=$([[ "${RVIZ_GUI}" == "1" ]] && echo true || echo false)"
  --env "USE_GAZEBO_GUI=$([[ "${GAZEBO_GUI}" == "1" ]] && echo true || echo false)"
  --env "AUTO_EXIT_DELAY=${AUTO_EXIT_DELAY}"
  --env "BENCHMARK_FILE=${BENCHMARK_CONTAINER_FILE}"
  --env "QT_X11_NO_MITSHM=1"
)

if [[ "${BENCHMARK}" == "1" ]]; then
  docker_args+=(--volume "${BENCHMARK_RUN_DIR}:/mighty_statistics:rw")
fi

if [[ -t 0 && -t 1 ]]; then
  docker_args+=(--tty)
fi

if [[ "${GPU}" == "1" ]]; then
  docker_args+=(--gpus all --env "NVIDIA_DRIVER_CAPABILITIES=all")
fi

for component in X Y Z; do
  variable="GOAL_${component}"
  if [[ -n "${!variable:-}" ]]; then
    docker_args+=(--env "${variable}=${!variable}")
  fi
done

if [[ "${X11_ENABLED}" == "1" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[ERROR] Enabling RViz or Gazebo GUI requires DISPLAY to be set." >&2
    echo "        Reconnect with X11 forwarding (ssh -X or ssh -Y)," >&2
    echo "        or run './run_gazebo_forest.sh 0 0'." >&2
    exit 1
  fi

  XAUTH_FILE="${XAUTHORITY:-${HOME}/.Xauthority}"
  if [[ ! -r "${XAUTH_FILE}" ]]; then
    echo "[ERROR] Enabling RViz or Gazebo GUI requires a readable Xauthority file." >&2
    echo "        Checked: ${XAUTH_FILE}" >&2
    echo "        Set XAUTHORITY to the file used by this MobaXterm session." >&2
    exit 1
  fi

  docker_args+=(
    --env "DISPLAY=${DISPLAY}"
    --env "XAUTHORITY=/root/.Xauthority"
    --volume "${XAUTH_FILE}:/root/.Xauthority:ro"
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
  )
fi

if [[ "${DEV}" == "1" ]]; then
  docker_args+=(
    --volume "${REPO_DIR}:/home/kkondo/code/mighty_ws/src/mighty"
    --volume "${REPO_DIR}/config:/home/kkondo/code/mighty_ws/install/mighty/share/mighty/config"
    --volume "${REPO_DIR}/launch:/home/kkondo/code/mighty_ws/install/mighty/share/mighty/launch"
    --volume "${REPO_DIR}/rviz:/home/kkondo/code/mighty_ws/install/mighty/share/mighty/rviz"
  )
fi

container_command=(
  -lc
  'source /opt/ros/humble/setup.bash
source /home/kkondo/code/decomp_ws/install/setup.bash
source /home/kkondo/code/mighty_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
cd /home/kkondo/code/mighty_ws

if [[ "${USE_RVIZ}" == "true" || "${USE_GAZEBO_GUI}" == "true" ]]; then
  if ! xdpyinfo >/dev/null 2>&1; then
    echo "[ERROR] The container cannot connect to DISPLAY=${DISPLAY:-<unset>}." >&2
    echo "        Check MobaXterm X11 forwarding and the host XAUTHORITY file." >&2
    exit 1
  fi
  echo "[MIGHTY] X11 preflight passed for DISPLAY=${DISPLAY}."
fi

args=(--mode gazebo -s /home/kkondo/code/mighty_ws/install/setup.bash --env "${ENV}" --auto-exit-delay "${AUTO_EXIT_DELAY}")
if [[ -n "${BENCHMARK_FILE}" ]]; then
  args+=(--benchmark-file "${BENCHMARK_FILE}")
fi
if [[ "${USE_RVIZ}" != "true" ]]; then
  args+=(--no-rviz)
fi
if [[ "${USE_GAZEBO_GUI}" == "true" ]]; then
  args+=(--gazebo-gui)
fi

goal_set=0
for component in X Y Z; do
  variable="GOAL_${component}"
  if [[ -n "${!variable:-}" ]]; then
    goal_set=1
  fi
done
if [[ "${goal_set}" == "1" ]]; then
  args+=(--goal "${GOAL_X:-305.0}" "${GOAL_Y:-0.0}" "${GOAL_Z:-3.0}")
fi

echo "[MIGHTY] python3 src/mighty/scripts/run_sim.py ${args[*]}"
exec python3 src/mighty/scripts/run_sim.py "${args[@]}"'
)

command=(docker "${docker_args[@]}" --entrypoint bash "${IMAGE}" "${container_command[@]}")

echo "[MIGHTY] World: ${WORLD}"
echo "[MIGHTY] RViz GUI: ${RVIZ_GUI}"
echo "[MIGHTY] Gazebo GUI: ${GAZEBO_GUI}"
echo "[MIGHTY] Auto-exit delay: ${AUTO_EXIT_DELAY} second(s)"
echo "[MIGHTY] Benchmark: ${BENCHMARK}"
if [[ "${BENCHMARK}" == "1" ]]; then
  echo "[MIGHTY] Benchmark output: ${BENCHMARK_RUN_DIR}"
fi
echo "[MIGHTY] Image: ${IMAGE}"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  printf '[DRY RUN] '
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

if [[ "${BENCHMARK}" != "1" ]]; then
  exec "${command[@]}"
fi

started_at="$(date --iso-8601=seconds)"
started_epoch="$(date +%s)"
set +e
"${command[@]}"
status=$?
set -e
ended_at="$(date --iso-8601=seconds)"
ended_epoch="$(date +%s)"
duration_s=$((ended_epoch - started_epoch))

ground_truth_generated=0
if [[ -s "${BENCHMARK_TRAJECTORY_FILE}" ]]; then
  world_file="${REPO_DIR}/worlds/${WORLD}.world"
  goal_args=()
  [[ -n "${GOAL_X:-}" ]] && goal_args+=(--goal-x "${GOAL_X}")
  [[ -n "${GOAL_Y:-}" ]] && goal_args+=(--goal-y "${GOAL_Y}")
  [[ -n "${GOAL_Z:-}" ]] && goal_args+=(--goal-z "${GOAL_Z}")
  if [[ -f "${world_file}" ]] && python3 "${REPO_DIR}/scripts/validate_trajectory_ground_truth.py" \
      --trajectory "${BENCHMARK_TRAJECTORY_FILE}" \
      --world "${world_file}" \
      --config "${REPO_DIR}/config/mighty.yaml" \
      --output "${BENCHMARK_GROUND_TRUTH_FILE}" \
      --mission-summary "${BENCHMARK_SUMMARY_FILE}" \
      --plot-output "${BENCHMARK_OVERLAY_FILE}" \
      --clearance-output "${BENCHMARK_CLEARANCE_FILE}" \
      "${goal_args[@]}"; then
    ground_truth_generated=1
  else
    echo "[WARN] Ground-truth trajectory validation did not complete." >&2
  fi
fi

benchmark_generated=0
if [[ -s "${BENCHMARK_HOST_FILE}" ]]; then
  benchmark_generated=1
elif (( status == 0 )); then
  echo "[ERROR] Simulation exited successfully but no benchmark CSV was generated." >&2
  status=3
fi

{
  printf 'run_id=%s\n' "${BENCHMARK_RUN_ID}"
  printf 'started_at=%s\n' "${started_at}"
  printf 'ended_at=%s\n' "${ended_at}"
  printf 'duration_s=%s\n' "${duration_s}"
  printf 'exit_code=%s\n' "${status}"
  printf 'benchmark_generated=%s\n' "${benchmark_generated}"
  printf 'ground_truth_generated=%s\n' "${ground_truth_generated}"
  printf 'world=%s\n' "${WORLD}"
  printf 'goal_x=%s\n' "${GOAL_X:-default}"
  printf 'goal_y=%s\n' "${GOAL_Y:-default}"
  printf 'goal_z=%s\n' "${GOAL_Z:-default}"
  printf 'auto_exit_delay=%s\n' "${AUTO_EXIT_DELAY}"
  printf 'image=%s\n' "${IMAGE}"
} > "${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_metadata.env"

exit "${status}"
