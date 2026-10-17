#!/usr/bin/env bash
# Run this in terminal C while the two sensor drivers run in A and B.
# Trace the entire launch tree so an incoming signal's sender PID is recorded.
source "$(dirname "${BASH_SOURCE[0]}")/calibration_env.sh"
set -eo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
config="$LIVO_WS/src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml"
test -f "$config"
command -v strace >/dev/null
run_dir=$(mktemp -d "$CALIB_ROOT/fusion_diagnostics_XXXXXXXX")
cp "$config" "$run_dir/loaded_config.yaml"
printf 'Diagnostic output: %s\n' "$run_dir"

# Record only process identities, not command arguments that may contain secrets.
(
  while true; do
    date --iso-8601=seconds
    ps -eo pid,ppid,pgid,sid,comm --no-headers
    sleep 1
  done
) > "$run_dir/processes.log" &
monitor_pid=$!
trap 'kill "$monitor_pid" 2>/dev/null || true; wait "$monitor_pid" 2>/dev/null || true' EXIT

# No timeout is applied to mapping. The trace includes si_pid/si_code when
# SIGINT/SIGTERM arrives, allowing external signals to be distinguished from
# a SIGSEGV raised by the mapper. Normal stdout/stderr is also retained.
set +e
nice -n 15 strace -f -tt -s 160 \
  -e trace=kill,tgkill,tkill \
  -e signal=SIGINT,SIGTERM,SIGSEGV,SIGABRT,SIGHUP \
  -o "$run_dir/signals.log" \
  ros2 launch fast_livo mapping_mid360s_d435i.launch.py \
    visual_config_file:="$config" use_rviz:=false \
  2>&1 | tee "$run_dir/console.log"
statuses=("${PIPESTATUS[@]}")
printf '%s\n' "${statuses[0]}" > "$run_dir/exit_code.txt"
printf 'Trace saved in: %s\n' "$run_dir"
exit "${statuses[0]}"
