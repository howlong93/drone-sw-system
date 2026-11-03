#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./run_gazebo_forest.sh [RVIZ_GUI] [GAZEBO_GUI]

Run the README "Single UAV through a Gazebo forest" scenario in Docker.

  RVIZ_GUI=1    Show RViz (default: 1).
  RVIZ_GUI=0    Do not start RViz.
  GAZEBO_GUI=1  Show the Gazebo client (default: 0).
  GAZEBO_GUI=0  Run Gazebo server-only.

Examples:
  ./run_gazebo_forest.sh        # RViz on, Gazebo GUI off
  ./run_gazebo_forest.sh 1 1    # RViz and Gazebo GUI on
  ./run_gazebo_forest.sh 0 1    # Gazebo GUI only
  ./run_gazebo_forest.sh 0 0    # Fully headless

Optional environment variables:
  MIGHTY_IMAGE   Docker image to run (default: mighty:latest)
  ENV            Gazebo world (default: hard_forest)
  GOAL_X/Y/Z     Goal components (default selected by run_sim.py: 305 0 3)
  GPU            Set to 0 to omit Docker GPU access (default: 1)
  DEV            Set to 1 to mount the current source/config/launch/rviz (default: 0)
  DRY_RUN        Set to 1 to print the Docker command without running it
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

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${MIGHTY_IMAGE:-mighty:latest}"
WORLD="${ENV:-hard_forest}"
GPU="${GPU:-1}"
DEV="${DEV:-0}"

if [[ "${GPU}" != "0" && "${GPU}" != "1" ]]; then
  echo "[ERROR] GPU must be 0 or 1 (received: ${GPU})." >&2
  exit 2
fi

if [[ "${DEV}" != "0" && "${DEV}" != "1" ]]; then
  echo "[ERROR] DEV must be 0 or 1 (received: ${DEV})." >&2
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

docker_args=(
  run
  --rm
  --name mighty-gazebo-forest
  --interactive
  --privileged
  --network=host
  --hostname "$(hostname)"
  --env "MODE=gazebo"
  --env "ENV=${WORLD}"
  --env "USE_RVIZ=$([[ "${RVIZ_GUI}" == "1" ]] && echo true || echo false)"
  --env "USE_GAZEBO_GUI=$([[ "${GAZEBO_GUI}" == "1" ]] && echo true || echo false)"
  --env "QT_X11_NO_MITSHM=1"
)

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

args=(--mode gazebo -s /home/kkondo/code/mighty_ws/install/setup.bash --env "${ENV}")
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
echo "[MIGHTY] Image: ${IMAGE}"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  printf '[DRY RUN] '
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

exec "${command[@]}"
