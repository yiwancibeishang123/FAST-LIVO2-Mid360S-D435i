# Source this file in each calibration terminal (Bash).
export LIVO_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export CALIB_ROOT="${CALIB_ROOT:-$LIVO_WS/calibration_data/rigid_mount_01}"
export CALIB_IMAGE="koide3/direct_visual_lidar_calibration:humble"
source /opt/ros/humble/setup.bash
export LIVOX_DRIVER_WS="${LIVOX_DRIVER_WS:-/home/hzw/livox_ws}"
source "$LIVOX_DRIVER_WS/install/setup.bash"
source "$LIVO_WS/install/setup.bash"
