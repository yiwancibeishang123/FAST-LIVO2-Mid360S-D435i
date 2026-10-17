#!/usr/bin/env bash
# Use in the desktop X11 session. Never commit docker.xauth.
source "$(dirname "${BASH_SOURCE[0]}")/calibration_env.sh"
set -eo pipefail
if [[ $# -lt 1 || -z "${DISPLAY:-}" ]]; then
  echo "Usage: bash scripts/calib_gui.sh initial_guess_manual|calibrate|viewer [options] (X11 desktop required)" >&2
  exit 1
fi
test -f "$CALIB_ROOT/preprocessed/calib.json"
export CALIB_XAUTH="$CALIB_ROOT/docker.xauth"
touch "$CALIB_XAUTH"
chmod 600 "$CALIB_XAUTH"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$CALIB_XAUTH" nmerge -
if [[ ! -s "$CALIB_XAUTH" ]]; then
  echo "X11 authorization is empty. Check DISPLAY and XAUTHORITY in your desktop terminal." >&2
  exit 1
fi
exec docker run --rm -it --network host --ipc host \
  --cpus=1 --memory=5g --memory-swap=5g \
  -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/calib.xauth \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$CALIB_XAUTH:/tmp/calib.xauth:ro" \
  -v "$CALIB_ROOT/preprocessed:/data" \
  "$CALIB_IMAGE" ros2 run direct_visual_lidar_calibration "$@" /data
