#!/usr/bin/env bash

set -Eeuo pipefail

# =============================================================================
# User configuration (edit these values for the next test batch)
# =============================================================================
TEST_RUN_COUNT="${TEST_RUN_COUNT:-5}"  # Number of simulation runs
RVIZ_GUI_DEFAULT="${RVIZ_GUI_DEFAULT:-0}"  # 1: enable RViz, 0: disable
GAZEBO_GUI_DEFAULT="${GAZEBO_GUI_DEFAULT:-0}"  # 1: enable Gazebo GUI, 0: disable
RECORD_STATISTICS="${RECORD_STATISTICS:-1}"  # 1: record benchmark data, 0: disable
ENABLE_SAFETY_GATE="${ENABLE_SAFETY_GATE:-0}"  # 1: reject unsafe candidates, 0: observe only
DEBUG_TESTING="${DEBUG_TESTING:-1}"  # 1: targeted corridor diagnostics; keep 0 for normal runs
SC_METHOD="${SC_METHOD:-pathcover_line_reuse}"  # ellipsoid, risp_point, risp_line, pathcover_line, pathcover_point, or pathcover_line_reuse
RISP_ALPHA="${RISP_ALPHA:-0.05}"  # RISP separation margin fraction in (0, 1)
SC_ELLIPSOID_FALLBACK="${SC_ELLIPSOID_FALLBACK:-0}"  # 1: ellipsoid fallback after RISP failure
RISP_RANDOM_SEED="${RISP_RANDOM_SEED:-1}"  # deterministic non-negative base seed
PATHCOVER_MAX_CORRIDORS="${PATHCOVER_MAX_CORRIDORS:-12}"
PATHCOVER_OVERLAP_DISTANCE="${PATHCOVER_OVERLAP_DISTANCE:-0.25}"  # metres
PATHCOVER_MIN_OVERLAP_MARGIN="${PATHCOVER_MIN_OVERLAP_MARGIN:-0.001}"  # metres
PATHCOVER_MIN_PROGRESS="${PATHCOVER_MIN_PROGRESS:-0.05}"  # metres; point PathCover only
PATHCOVER_POINT_MAX_RETRIES="${PATHCOVER_POINT_MAX_RETRIES:-3}"
# 1 enables the older heavy O(corridors*obstacles) audit. Lightweight P0/P1/P2
# recording is always enabled whenever RECORD_STATISTICS=1.
BENCHMARK_REPLAN_DETAILS="${BENCHMARK_REPLAN_DETAILS:-0}"
STATISTICS_FOLDER_NAME="${STATISTICS_FOLDER_NAME:-${SC_METHOD}_a0.05_$(date +%Y%m%d_%H%M%S)}"
RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-280}"  # Overall timeout for each run
FLIGHT_TIMEOUT_SECONDS="${FLIGHT_TIMEOUT_SECONDS:-240}"  # Timeout after flight starts

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# With no arguments, launch the configured batch. Set TEST_RUN_COUNT=1 above
# for a single trial. The batch runner calls this script with explicit GUI
# arguments, so this branch cannot recurse.
if (( $# == 0 )); then
  export BATCH_ID="${BATCH_ID:-${STATISTICS_FOLDER_NAME}}"
  export BATCH_RVIZ_GUI="${RVIZ_GUI_DEFAULT}"
  export BATCH_GAZEBO_GUI="${GAZEBO_GUI_DEFAULT}"
  export BATCH_RECORD_STATISTICS="${RECORD_STATISTICS}"
  export BATCH_BENCHMARK_REPLAN_DETAILS="${BENCHMARK_REPLAN_DETAILS}"
  export BATCH_ENABLE_SAFETY_GATE="${ENABLE_SAFETY_GATE}"
  export BATCH_DEBUG_TESTING="${DEBUG_TESTING}"
  export BATCH_SC_METHOD="${SC_METHOD}"
  export BATCH_RISP_ALPHA="${RISP_ALPHA}"
  export BATCH_SC_ELLIPSOID_FALLBACK="${SC_ELLIPSOID_FALLBACK}"
  export BATCH_RISP_RANDOM_SEED="${RISP_RANDOM_SEED}"
  export BATCH_PATHCOVER_MAX_CORRIDORS="${PATHCOVER_MAX_CORRIDORS}"
  export BATCH_PATHCOVER_OVERLAP_DISTANCE="${PATHCOVER_OVERLAP_DISTANCE}"
  export BATCH_PATHCOVER_MIN_OVERLAP_MARGIN="${PATHCOVER_MIN_OVERLAP_MARGIN}"
  export BATCH_PATHCOVER_MIN_PROGRESS="${PATHCOVER_MIN_PROGRESS}"
  export BATCH_PATHCOVER_POINT_MAX_RETRIES="${PATHCOVER_POINT_MAX_RETRIES}"
  export TIMEOUT_S="${RUN_TIMEOUT_SECONDS}"
  export FLIGHT_TIMEOUT_S="${FLIGHT_TIMEOUT_SECONDS}"
  exec "${REPO_DIR}/run_benchmark_batch_drone.sh" "${TEST_RUN_COUNT}"
fi

usage() {
  cat <<'EOF'
Usage: ./run_gazebo_forest.sh [RVIZ_GUI] [GAZEBO_GUI]

With no arguments, run the benchmark batch configured at the top of this file.
With GUI arguments, run one README "Single UAV through a Gazebo forest" trial.

  RVIZ_GUI=1    Show RViz (default is configured at the top of this file).
  RVIZ_GUI=0    Do not start RViz.
  GAZEBO_GUI=1  Show the Gazebo client (default is configured at the top of this file).
  GAZEBO_GUI=0  Run Gazebo server-only.

Examples:
  ./run_gazebo_forest.sh        # configured benchmark batch
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
  BENCHMARK       Override RECORD_STATISTICS for this invocation
  BENCHMARK_REPLAN_DETAILS  1: enable heavy per-replan audits; 0: lightweight only
  ENABLE_SAFETY_GATE  1: reject unsafe candidates; 0: record and publish for A/B tests
  DEBUG_TESTING   1: enable targeted PathCover/corridor diagnosis; 0: normal flight
  SC_METHOD       Safe-corridor generator: ellipsoid, risp_point, risp_line, pathcover_line, pathcover_point, or pathcover_line_reuse
  RISP_ALPHA      RISP separation margin fraction in (0, 1)
  SC_ELLIPSOID_FALLBACK  1: use ellipsoid after a RISP failure; 0: pure RISP
  RISP_RANDOM_SEED Deterministic non-negative RISP base seed
  PATHCOVER_MAX_CORRIDORS Maximum number of PathCover corridors
  PATHCOVER_OVERLAP_DISTANCE Backtrack distance used to create overlap (metres)
  PATHCOVER_MIN_OVERLAP_MARGIN Required normalized seam margin (metres)
  PATHCOVER_MIN_PROGRESS Required new point-corridor coverage (metres)
  PATHCOVER_POINT_MAX_RETRIES Deterministic retries after the initial point attempt
  BENCHMARK_RUN_ID Identifier used for the output folder and CSV filename
  BENCHMARK_RUN_DIR Host output directory; defaults to statistics/<run-id>
  CONTAINER_NAME  Docker container name (default: mighty-gazebo-forest)
  DRY_RUN        Set to 1 to print the Docker command without running it
EOF
}

RVIZ_GUI="${1:-${RVIZ_GUI_DEFAULT}}"
GAZEBO_GUI="${2:-${GAZEBO_GUI_DEFAULT}}"

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

if [[ ! "${TEST_RUN_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] TEST_RUN_COUNT must be a positive integer (received: ${TEST_RUN_COUNT})." >&2
  exit 2
fi

if [[ "${RECORD_STATISTICS}" != "0" && "${RECORD_STATISTICS}" != "1" ]]; then
  echo "[ERROR] RECORD_STATISTICS must be 0 or 1 (received: ${RECORD_STATISTICS})." >&2
  exit 2
fi

if [[ "${BENCHMARK_REPLAN_DETAILS}" != "0" && "${BENCHMARK_REPLAN_DETAILS}" != "1" ]]; then
  echo "[ERROR] BENCHMARK_REPLAN_DETAILS must be 0 or 1 (received: ${BENCHMARK_REPLAN_DETAILS})." >&2
  exit 2
fi

if [[ "${ENABLE_SAFETY_GATE}" != "0" && "${ENABLE_SAFETY_GATE}" != "1" ]]; then
  echo "[ERROR] ENABLE_SAFETY_GATE must be 0 or 1 (received: ${ENABLE_SAFETY_GATE})." >&2
  exit 2
fi

if [[ "${DEBUG_TESTING}" != "0" && "${DEBUG_TESTING}" != "1" ]]; then
  echo "[ERROR] DEBUG_TESTING must be 0 or 1 (received: ${DEBUG_TESTING})." >&2
  exit 2
fi
if [[ "${DEBUG_TESTING}" == "1" && "${RECORD_STATISTICS}" != "1" ]]; then
  echo "[ERROR] DEBUG_TESTING=1 requires RECORD_STATISTICS=1." >&2
  exit 2
fi

if [[ "${SC_METHOD}" != "ellipsoid" && "${SC_METHOD}" != "risp_point" &&
      "${SC_METHOD}" != "risp_line" && "${SC_METHOD}" != "pathcover_line" &&
      "${SC_METHOD}" != "pathcover_point" &&
      "${SC_METHOD}" != "pathcover_line_reuse" ]]; then
  echo "[ERROR] SC_METHOD is invalid (received: ${SC_METHOD})." >&2
  exit 2
fi
if ! awk -v value="${RISP_ALPHA}" 'BEGIN { exit !(value > 0 && value < 1) }'; then
  echo "[ERROR] RISP_ALPHA must be numeric and in (0, 1) (received: ${RISP_ALPHA})." >&2
  exit 2
fi
if [[ "${SC_ELLIPSOID_FALLBACK}" != "0" && "${SC_ELLIPSOID_FALLBACK}" != "1" ]]; then
  echo "[ERROR] SC_ELLIPSOID_FALLBACK must be 0 or 1 (received: ${SC_ELLIPSOID_FALLBACK})." >&2
  exit 2
fi
if [[ ! "${RISP_RANDOM_SEED}" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] RISP_RANDOM_SEED must be a non-negative integer (received: ${RISP_RANDOM_SEED})." >&2
  exit 2
fi
[[ "${PATHCOVER_MAX_CORRIDORS}" =~ ^([2-9]|[1-9][0-9]+)$ ]] || { echo "[ERROR] PATHCOVER_MAX_CORRIDORS must be at least 2." >&2; exit 2; }
awk -v value="${PATHCOVER_OVERLAP_DISTANCE}" 'BEGIN { exit !(value >= 0) }' || { echo "[ERROR] PATHCOVER_OVERLAP_DISTANCE must be non-negative." >&2; exit 2; }
awk -v value="${PATHCOVER_MIN_OVERLAP_MARGIN}" 'BEGIN { exit !(value >= 0) }' || { echo "[ERROR] PATHCOVER_MIN_OVERLAP_MARGIN must be non-negative." >&2; exit 2; }
awk -v value="${PATHCOVER_MIN_PROGRESS}" 'BEGIN { exit !(value > 0) }' || { echo "[ERROR] PATHCOVER_MIN_PROGRESS must be positive." >&2; exit 2; }
[[ "${PATHCOVER_POINT_MAX_RETRIES}" =~ ^[0-9]+$ ]] || { echo "[ERROR] PATHCOVER_POINT_MAX_RETRIES must be non-negative." >&2; exit 2; }

if [[ ! "${STATISTICS_FOLDER_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "[ERROR] STATISTICS_FOLDER_NAME contains unsupported characters: ${STATISTICS_FOLDER_NAME}" >&2
  exit 2
fi

for timeout_setting in RUN_TIMEOUT_SECONDS FLIGHT_TIMEOUT_SECONDS; do
  if [[ ! "${!timeout_setting}" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]]; then
    echo "[ERROR] ${timeout_setting} must be a positive number (received: ${!timeout_setting})." >&2
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
FORCE_TTY="${FORCE_TTY:-0}"
BENCHMARK="${BENCHMARK:-${RECORD_STATISTICS}}"
CONTAINER_NAME="${CONTAINER_NAME:-mighty-gazebo-forest}"

if [[ "${GPU}" != "0" && "${GPU}" != "1" ]]; then
  echo "[ERROR] GPU must be 0 or 1 (received: ${GPU})." >&2
  exit 2
fi

if [[ "${FORCE_TTY}" != "0" && "${FORCE_TTY}" != "1" ]]; then
  echo "[ERROR] FORCE_TTY must be 0 or 1 (received: ${FORCE_TTY})." >&2
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
  BENCHMARK_RUN_ID="${BENCHMARK_RUN_ID:-${STATISTICS_FOLDER_NAME}}"
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
  --privileged
  --network=host
  --hostname "$(hostname)"
  --env "MODE=gazebo"
  --env "ENV=${WORLD}"
  --env "USE_RVIZ=$([[ "${RVIZ_GUI}" == "1" ]] && echo true || echo false)"
  --env "USE_GAZEBO_GUI=$([[ "${GAZEBO_GUI}" == "1" ]] && echo true || echo false)"
  --env "AUTO_EXIT_DELAY=${AUTO_EXIT_DELAY}"
  --env "BENCHMARK_FILE=${BENCHMARK_CONTAINER_FILE}"
  --env "BENCHMARK_REPLAN_DETAILS=${BENCHMARK_REPLAN_DETAILS}"
  --env "ENABLE_SAFETY_GATE=${ENABLE_SAFETY_GATE}"
  --env "DEBUG_TESTING=${DEBUG_TESTING}"
  --env "SC_METHOD=${SC_METHOD}"
  --env "RISP_ALPHA=${RISP_ALPHA}"
  --env "SC_ELLIPSOID_FALLBACK=${SC_ELLIPSOID_FALLBACK}"
  --env "RISP_RANDOM_SEED=${RISP_RANDOM_SEED}"
  --env "PATHCOVER_MAX_CORRIDORS=${PATHCOVER_MAX_CORRIDORS}"
  --env "PATHCOVER_OVERLAP_DISTANCE=${PATHCOVER_OVERLAP_DISTANCE}"
  --env "PATHCOVER_MIN_OVERLAP_MARGIN=${PATHCOVER_MIN_OVERLAP_MARGIN}"
  --env "PATHCOVER_MIN_PROGRESS=${PATHCOVER_MIN_PROGRESS}"
  --env "PATHCOVER_POINT_MAX_RETRIES=${PATHCOVER_POINT_MAX_RETRIES}"
  --env "QT_X11_NO_MITSHM=1"
)

if [[ "${BENCHMARK}" == "1" ]]; then
  docker_args+=(--volume "${BENCHMARK_RUN_DIR}:/mighty_statistics:rw")
fi

if [[ "${FORCE_TTY}" != "1" ]]; then
  docker_args+=(--interactive)
fi

if [[ "${FORCE_TTY}" == "1" || ( -t 0 && -t 1 ) ]]; then
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
args+=(--sc-method "${SC_METHOD}" --risp-alpha "${RISP_ALPHA}" --risp-random-seed "${RISP_RANDOM_SEED}")
args+=(--pathcover-max-corridors "${PATHCOVER_MAX_CORRIDORS}" --pathcover-overlap-distance "${PATHCOVER_OVERLAP_DISTANCE}" --pathcover-min-overlap-margin "${PATHCOVER_MIN_OVERLAP_MARGIN}" --pathcover-min-progress "${PATHCOVER_MIN_PROGRESS}" --pathcover-point-max-retries "${PATHCOVER_POINT_MAX_RETRIES}")
if [[ "${SC_ELLIPSOID_FALLBACK}" == "1" ]]; then
  args+=(--sc-ellipsoid-fallback)
fi
if [[ -n "${BENCHMARK_FILE}" ]]; then
  args+=(--benchmark-file "${BENCHMARK_FILE}")
  if [[ "${BENCHMARK_REPLAN_DETAILS}" == "1" ]]; then
    args+=(--benchmark-replan-details)
  fi
fi
if [[ "${ENABLE_SAFETY_GATE}" != "1" ]]; then
  args+=(--disable-safety-gate)
fi
if [[ "${DEBUG_TESTING}" == "1" ]]; then
  args+=(--debug-testing)
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
echo "[MIGHTY] Heavy replan details: ${BENCHMARK_REPLAN_DETAILS}"
echo "[MIGHTY] Safety gate: ${ENABLE_SAFETY_GATE}"
echo "[MIGHTY] Debug testing: ${DEBUG_TESTING}"
echo "[MIGHTY] SC method: ${SC_METHOD}"
echo "[MIGHTY] RISP alpha: ${RISP_ALPHA}"
echo "[MIGHTY] Ellipsoid fallback: ${SC_ELLIPSOID_FALLBACK}"
echo "[MIGHTY] RISP random seed: ${RISP_RANDOM_SEED}"
echo "[MIGHTY] PathCover max corridors: ${PATHCOVER_MAX_CORRIDORS}"
echo "[MIGHTY] PathCover overlap distance: ${PATHCOVER_OVERLAP_DISTANCE}"
echo "[MIGHTY] PathCover minimum overlap margin: ${PATHCOVER_MIN_OVERLAP_MARGIN}"
echo "[MIGHTY] PathCover minimum point progress: ${PATHCOVER_MIN_PROGRESS}"
echo "[MIGHTY] PathCover point max retries: ${PATHCOVER_POINT_MAX_RETRIES}"
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

# Preserve the exact run inputs. These operations happen only after flight.
cp "${REPO_DIR}/config/mighty.yaml" \
  "${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_mighty.yaml"
git_commit="$(git -C "${REPO_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
git_dirty=0
[[ -z "$(git -C "${REPO_DIR}" status --porcelain 2>/dev/null)" ]] || git_dirty=1
git_diff_sha256="$(git -C "${REPO_DIR}" diff --binary HEAD 2>/dev/null | sha256sum | awk '{print $1}')"
config_sha256="$(sha256sum "${REPO_DIR}/config/mighty.yaml" | awk '{print $1}')"
world_sha256="$(sha256sum "${REPO_DIR}/worlds/${WORLD}.world" | awk '{print $1}')"
image_id="$(docker image inspect --format '{{.Id}}' "${IMAGE}" 2>/dev/null || echo unknown)"
planner_co="$(awk '$1 == "planner_Co:" {print $2; exit}' "${REPO_DIR}/config/mighty.yaml")"
inflation_hgp="$(awk '$1 == "inflation_hgp:" {print $2; exit}' "${REPO_DIR}/config/mighty.yaml")"
drone_bbox="$(sed -n 's/^[[:space:]]*drone_bbox:[[:space:]]*\([^#]*\).*$/\1/p' \
  "${REPO_DIR}/config/mighty.yaml" | tr -d '[] ' )"

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
  printf 'image_id=%s\n' "${image_id}"
  printf 'git_commit=%s\n' "${git_commit}"
  printf 'git_dirty=%s\n' "${git_dirty}"
  printf 'git_diff_sha256=%s\n' "${git_diff_sha256}"
  printf 'config_sha256=%s\n' "${config_sha256}"
  printf 'world_sha256=%s\n' "${world_sha256}"
  printf 'benchmark_replan_details=%s\n' "${BENCHMARK_REPLAN_DETAILS}"
  printf 'enable_safety_gate=%s\n' "${ENABLE_SAFETY_GATE}"
  printf 'debug_testing=%s\n' "${DEBUG_TESTING}"
  printf 'sc_method=%s\n' "${SC_METHOD}"
  printf 'risp_alpha=%s\n' "${RISP_ALPHA}"
  printf 'sc_ellipsoid_fallback=%s\n' "${SC_ELLIPSOID_FALLBACK}"
  printf 'risp_random_seed=%s\n' "${RISP_RANDOM_SEED}"
  printf 'pathcover_max_corridors=%s\n' "${PATHCOVER_MAX_CORRIDORS}"
  printf 'pathcover_overlap_distance=%s\n' "${PATHCOVER_OVERLAP_DISTANCE}"
  printf 'pathcover_min_overlap_margin=%s\n' "${PATHCOVER_MIN_OVERLAP_MARGIN}"
  printf 'pathcover_min_progress=%s\n' "${PATHCOVER_MIN_PROGRESS}"
  printf 'pathcover_point_max_retries=%s\n' "${PATHCOVER_POINT_MAX_RETRIES}"
  printf 'planner_Co=%s\n' "${planner_co}"
  printf 'inflation_hgp=%s\n' "${inflation_hgp}"
  printf 'drone_bbox=%s\n' "${drone_bbox}"
} > "${BENCHMARK_RUN_DIR}/${BENCHMARK_RUN_ID}_metadata.env"

exit "${status}"
