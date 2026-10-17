#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/calibration_env.sh"
set -eo pipefail
name="${1:-}"
if [[ ! "$name" =~ ^pose_0[1-5]$ ]]; then
  echo "Usage: bash scripts/record_calibration_pose.sh pose_01 (through pose_05)" >&2
  exit 1
fi
mkdir -p "$CALIB_ROOT/bags"
if [[ -e "$CALIB_ROOT/bags/$name" ]]; then
  echo "Already exists: $CALIB_ROOT/bags/$name. Choose a fresh session; do not overwrite." >&2
  exit 1
fi
set +e
timeout --signal=INT --kill-after=10s 20s \
  ros2 bag record --storage sqlite3 --max-cache-size 33554432 \
  -o "$CALIB_ROOT/bags/$name" /livox/lidar /livox/imu \
  /camera/camera/color/image_raw /camera/camera/color/camera_info
status=$?
set -e
if [[ $status -ne 0 && $status -ne 124 ]]; then
  echo "Recorder failed with status $status; preserve files and inspect output." >&2
  exit "$status"
fi
ros2 bag info "$CALIB_ROOT/bags/$name"
