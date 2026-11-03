#!/usr/bin/env bash
set -uo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$repo_dir"

count=${1:-20}

# Batch controls. Override them on the command line, for example:
#   GUI=0 USE_LINE_SEED_RISP=1 LINE_SEED_MAX_LENGTH=1.5 ./run_benchmark_batch_drone.sh 10
gui_flag=${GUI:-0}
use_line_seed_risp=${USE_LINE_SEED_RISP:-1}
line_seed_max_length=${LINE_SEED_MAX_LENGTH:-1.0}

start_x=${START_X:--18.5}
start_y=${START_Y:-8.5}
goal_x=${GOAL_X:-9.0}
goal_y=${GOAL_Y:--8.5}
timeout_s=${TIMEOUT_S:-120.0}
if [[ $use_line_seed_risp == 1 ]]; then
  default_prefix="lseed_${start_x}_${start_y}_to_${goal_x}_${goal_y}"
else
  default_prefix="pseed_${start_x}_${start_y}_to_${goal_x}_${goal_y}"
fi
run_prefix=${RUN_PREFIX:-$default_prefix}
batch_id=${BATCH_ID:-$(date +%Y%m%d_%H%M%S)}

[[ $count =~ ^[1-9][0-9]*$ ]] || { echo "count must be a positive integer" >&2; exit 2; }
[[ $gui_flag == 0 || $gui_flag == 1 ]] || { echo "GUI must be 0 or 1" >&2; exit 2; }
[[ $use_line_seed_risp == 0 || $use_line_seed_risp == 1 ]] || { echo "USE_LINE_SEED_RISP must be 0 or 1" >&2; exit 2; }
[[ $line_seed_max_length =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "LINE_SEED_MAX_LENGTH must be a positive number" >&2; exit 2; }
[[ ! $line_seed_max_length =~ ^0+([.]0+)?$ ]] || { echo "LINE_SEED_MAX_LENGTH must be greater than zero" >&2; exit 2; }
[[ $run_prefix =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "RUN_PREFIX contains unsupported characters" >&2; exit 2; }

echo "Batch configuration:"
echo "  GUI                  : $gui_flag"
echo "  USE_LINE_SEED_RISP   : $use_line_seed_risp"
echo "  LINE_SEED_MAX_LENGTH : $line_seed_max_length"

failures=0
for ((index = 1; index <= count; index++)); do
  run_id=$(printf '%s_%s_run_%02d' "$run_prefix" "$batch_id" "$index")
  printf '\n===== Batch run %d/%d: %s =====\n' "$index" "$count" "$run_id"

  if START_X="$start_x" START_Y="$start_y" \
     GOAL_X="$goal_x" GOAL_Y="$goal_y" TIMEOUT_S="$timeout_s" \
     GUI="$gui_flag" USE_LINE_SEED_RISP="$use_line_seed_risp" \
     LINE_SEED_MAX_LENGTH="$line_seed_max_length" \
     ./run_benchmark_drone.sh "$run_id"; then
    printf '===== Completed %s =====\n' "$run_id"
  else
    status=$?
    printf '===== Failed %s (exit %d) =====\n' "$run_id" "$status" >&2
    failures=$((failures + 1))
  fi
done

printf '\nBatch finished: %d requested, %d command failures.\n' "$count" "$failures"
((failures == 0))
