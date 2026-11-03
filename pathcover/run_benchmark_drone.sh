#!/usr/bin/env bash
set -Eeuo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$repo_dir"

image=${PATHCOVER_IMAGE:-ros-noetic-pathcover:latest}
run_id=${1:-quadrotor_left_top_to_right_room_left_bottom_$(date +%Y%m%d_%H%M%S)}
start_x=${START_X:--18.5}
start_y=${START_Y:-8.5}
goal_x=${GOAL_X:-9.0}
goal_y=${GOAL_Y:--8.5}
timeout_s=${TIMEOUT_S:-240.0}
gui_flag=${GUI:-0}
use_line_seed_risp=${USE_LINE_SEED_RISP:-1}
line_seed_max_length=${LINE_SEED_MAX_LENGTH:-1.0}

if [[ ! $run_id =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "run_id 只能包含英文字母、數字、底線、句點與連字號" >&2
  exit 2
fi
for value in "$start_x" "$start_y" "$goal_x" "$goal_y" "$timeout_s"; do
  if [[ ! $value =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "座標與 timeout 必須是數字：$value" >&2
    exit 2
  fi
done
[[ $gui_flag == 0 || $gui_flag == 1 ]] || { echo "GUI 必須是 0 或 1" >&2; exit 2; }
[[ $use_line_seed_risp == 0 || $use_line_seed_risp == 1 ]] || { echo "USE_LINE_SEED_RISP 必須是 0 或 1" >&2; exit 2; }
[[ $line_seed_max_length =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "LINE_SEED_MAX_LENGTH 必須是正數" >&2; exit 2; }
[[ ! $line_seed_max_length =~ ^0+([.]0+)?$ ]] || { echo "LINE_SEED_MAX_LENGTH 必須大於 0" >&2; exit 2; }

if [[ $use_line_seed_risp == 1 ]]; then
  use_line_seed_bool=true
else
  use_line_seed_bool=false
fi

case_dir="start_${start_x}_${start_y}__goal_${goal_x}_${goal_y}"
host_output_dir="$repo_dir/benchmark_results/$case_dir"
container_output_dir="/home/PathCover/PathCover_ws/benchmark_results/$case_dir"

if ! docker image inspect "$image" >/dev/null 2>&1; then
  echo "找不到映像：$image" >&2
  exit 1
fi
if docker container inspect pathcover-drone-benchmark >/dev/null 2>&1; then
  echo "已存在名為 pathcover-drone-benchmark 的容器；請先確認它是否仍在執行。" >&2
  exit 1
fi

mkdir -p "$host_output_dir"

docker_tty_args=()
if [[ -t 0 && -t 1 ]]; then
  docker_tty_args=(-it)
fi

gui=false
docker_gui_args=()
if [[ $gui_flag == 1 ]]; then
  if [[ -z ${DISPLAY:-} || ! -f ${HOME}/.Xauthority ]]; then
    echo "GUI=1 但未偵測到 DISPLAY 或 ~/.Xauthority；請使用 X11 forwarding，或改設 GUI=0。" >&2
    exit 2
  fi
  gui=true
  docker_gui_args+=(
    -e "DISPLAY=$DISPLAY"
    -e XAUTHORITY=/root/.Xauthority
    -e QT_X11_NO_MITSHM=1
    --mount "type=bind,src=${HOME}/.Xauthority,dst=/root/.Xauthority,readonly"
  )
fi

echo "Run ID : $run_id"
echo "Start  : ($start_x, $start_y)"
echo "Goal   : ($goal_x, $goal_y)"
echo "GUI    : $gui"
echo "Line seed RISP : $use_line_seed_bool"
echo "Line seed max  : $line_seed_max_length m"
echo "Output : $host_output_dir/$run_id"

docker run "${docker_tty_args[@]}" --rm \
  --name pathcover-drone-benchmark \
  --gpus all \
  --network host \
  --ipc host \
  --user 0:0 \
  "${docker_gui_args[@]}" \
  --mount "type=bind,src=$repo_dir,dst=/home/PathCover/PathCover_ws" \
  -w /home/PathCover/PathCover_ws \
  "$image" \
  bash -lc '
    set -Eeuo pipefail
    source /opt/ros/noetic/setup.bash
    cd /home/PathCover/PathCover_ws
    catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -DPATHCOVER_EXAMPLE=quadrotor
    source devel/setup.bash
    roslaunch corridor_planning benchmark_drone.launch \
      run_id:="$1" start_x:="$2" start_y:="$3" \
      goal_x:="$4" goal_y:="$5" timeout_s:="$6" \
      gui:="$7" rviz:="$7" output_dir:="$8" \
      use_line_seed_risp:="$9" line_seed_max_length:="${10}"
  ' _ "$run_id" "$start_x" "$start_y" "$goal_x" "$goal_y" "$timeout_s" "$gui" "$container_output_dir" "$use_line_seed_bool" "$line_seed_max_length"

echo
echo "Benchmark 完成：$host_output_dir/$run_id"
if [[ -f "$host_output_dir/$run_id/summary.json" ]]; then
  sed -n '1,240p' "$host_output_dir/$run_id/summary.json"
fi
