# Mid-360S + D435i live setup

The supplied launch starts in **LIO-only** mode. This checks the community FAST-LIVO2
ROS 2 port with `/livox/lidar` and `/livox/imu`. It does not count as visual
FAST-LIVO2 reproduction. No camera-to-LiDAR extrinsic has been measured yet.
As of 2026-10-17, the user has rigidly mounted the D435i to the LiDAR.
Extrinsics for this installation still need measurement; follow
`MID360S_D435I_CALIBRATION_EXECUTION.md`. The earlier optional visual smoke
test checked software execution only and cannot validate map geometry.

## Inputs

- LiDAR: `192.168.1.114`; host Ethernet: `192.168.1.50/24`.
- D435i serial: `261722072542`; color image:
  `/camera/camera/color/image_raw`, `640x480@30`, RGB8. Intrinsics from
  `/camera/camera/color/camera_info` are in
  `src/FAST-LIVO2/config/camera_d435i_261722072542.yaml`.
- LIO configuration: `src/FAST-LIVO2/config/mid360s_d435i_lio.yaml`.
  The LiDAR-to-IMU translation uses the FAST-LIO Mid-360 reference value;
  confirm it for this Mid-360S device before evaluating trajectory accuracy.

## Start, in three terminals

Source `/opt/ros/humble/setup.bash` in every terminal. Start the existing
Livox driver from `/home/hzw/livox_ws/install/setup.bash`, using
`ros2 launch livox_ros_driver2 msg_MID360s_launch.py`. Start the camera with:

```bash
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=false enable_sync:=false \
  align_depth.enable:=false enable_color:=true enable_depth:=false \
  enable_infra:=false enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=640x480x30
```

Then source this workspace's `install/setup.bash` and run:

```bash
OMP_NUM_THREADS=1 nice -n 15 ros2 launch fast_livo mapping_mid360s_d435i.launch.py use_rviz:=false
```

Avoid running FAST-LIO2 and FAST-LIVO2 mapping nodes at the same time on this
computer. The launch starts no duplicate sensor drivers or image republisher,
and RViz is off by default. `pcd_save_en` is false; the default single-file PCD
mode can grow without bound.

## Before visual mode

1. Rigidly mount the camera and LiDAR. Keep the mount fixed throughout calibration
   and data capture. Record the camera optical frame, not a camera body frame.
2. Calibrate the D435i color camera against the LiDAR. The program's `Rcl` and
   `Pcl` transform LiDAR points into the **color optical frame**:
   `p_color = Rcl * p_lidar + Pcl`. Confirm this convention by projecting LiDAR
   edge points onto several color images at different ranges.
3. Check image, LiDAR, and IMU timestamps on one clock. Start with
   `time_offset.img_time_offset: 0.0`; estimate any real capture-time offset
   from motion, rather than copying the Avia dataset's `0.1` seconds.
4. Copy `mid360s_d435i_lio.yaml` to a separate visual YAML. Set
   `common.img_en: 1`, insert measured `Rcl` (row-major 3x3) and `Pcl` (metres),
   then launch with `visual_config_file:=/absolute/path/to/your_visual.yaml`.
   The launch rejects the uncalibrated identity/zero placeholders.
5. Compare the visual result with LIO-only: verify image frames are used,
   projected points align with visible edges, odometry continues during
   movement, and maps do not double or smear. Record CPU, memory, and topic rates.

For a **short software-only visual smoke test** before rigid mounting, use the
explicit `mid360s_d435i_visual_smoke.yaml` file and
`allow_uncalibrated_visual_smoke:=true`. Its identity/zero camera extrinsics
are placeholders. Visual odometry or colored maps produced this way have no
geometric validity and should not be compared with LIO mapping quality.

```bash
OMP_NUM_THREADS=1 nice -n 15 ros2 launch fast_livo mapping_mid360s_d435i.launch.py \
  visual_config_file:="$(realpath src/FAST-LIVO2/config/mid360s_d435i_visual_smoke.yaml)" \
  allow_uncalibrated_visual_smoke:=true use_rviz:=false
```

On 2026-10-02 an 18-second visual smoke run subscribed to the color image
and executed 344 VIO steps and 345 LIO voxel-map updates. Odometry appeared
at about 20 Hz. The run had no continuing sync or timestamp warnings, but
**all visual-map additions were zero**. This means the image code path ran,
while actual visual constraints and a fused map have **not** been verified.
With the camera unmounted and the identity/zero placeholders, no map-quality
conclusion is possible. The launch refuses this smoke config unless the
explicit opt-in argument above is provided.

The source is the community ROS 2 `soft-sync` branch, not the official HKU-MARS
ROS 1 implementation. Its software time alignment is a starting point for
debugging, not proof that the three sensor clocks are synchronized.

## First live check (2026-10-02)

With RViz off and one OpenMP thread, the LiDAR published at 10 Hz, IMU at
about 200 Hz, and color camera at about 30 Hz. The LIO-only mapper published
`/aft_mapped_to_init` and `/cloud_registered` at about 10 Hz during a 35-second
smoke test. The initial IMU reset message was expected; no continuing time
sync or loop-back warning appeared. This checks the data path, not map accuracy.

One Mid-360S frame held 20,064 points with exactly four scan lines. Only
10,008 had nonzero range; 7,431 were at least 0.5 m away. Sampling every third
raw point and applying the range cutoff gave about 2,468 points per frame.
Sparse output in this mode is therefore chiefly zero returns, near-range
rejection, and deliberate sampling,
not an incorrect scan-line count. Once stable, try `point_filter_num: 2` then
`1` and compare CPU load and map quality. Keep PCD saving off during long runs.
