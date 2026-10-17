> **历史说明，包含当时的旧数据/旧初值状态。** 当前逐步操作统一看 [标定手册](docs/CALIBRATION_README.md)；运行融合看 [调试手册](docs/FASTLIVO2_DEBUG_README.md)。

# Mid-360S + D435i 安装、标定与 FAST-LIVO2 验证清单

适用工程：本目录的 ROS 2 Humble `soft-sync` 分支；雷达 `192.168.1.114`，主机
`192.168.1.50`，D435i 序列号 `261722072542`。2026-10-17 用户已确认两者刚性固定，
本次安装的外参尚待实测。按最新逐终端说明
[标定执行步骤](MID360S_D435I_CALIBRATION_EXECUTION.md) 操作。不要在标定后改变支架。
本配置使用雷达内置 IMU 的 `/livox/imu` 与 D435i 的彩色图像；不需要启用
D435i 的深度、陀螺仪或加速度计话题。

## 1. 先决定安装方向

- Mid-360S 官方视场为水平 360°、垂直约 `−7°～+52°`；这个不对称的垂直视场
  相对于**雷达自身的安装方向**，并非总是世界坐标的上下方向。
- 当前 D435i RGB `640×480` 的 `fx=608.16, fy=607.04`，计算得到近似水平
  `55.5°`、垂直 `43.1°`，半高约 `21.6°`。这里使用 **RGB 彩色相机**视场，
  不要拿 D435i 的深度/红外视场代替。
- 在相机朝向的雷达方位截面中，若希望整幅 RGB 图像落在雷达垂直视场内，
  相机中心视线应向雷达 `+52°` 那侧偏约 `15°～30°`；先以 `20°` 作机械初值。
  这是视场重叠的几何建议，不是标定外参，也不是所有场景的精度最优角。
- 室内前向看墙/通道的例子：雷达朝前的水平参考方向下俯约 `20°`，相机大致
  水平朝前，两者方位角尽量一致。此时相机相对雷达朝宽视场侧偏约 `20°`。
  如果需要更多地面，则让**整个方案**一起更向下，重新检查图像中墙面、地面
  和点云的共同区域。倒装时宽视场侧会向下；倒装与离地间隙请核对雷达手册。
- 更直接的验收法：让有墙角、门框、桌椅的场景同时出现在 RGB 图像和点云内，
  共同区域尽量覆盖画面中央与上下边缘；不要让螺旋桨、支架、电缆遮挡扫描。

| 目标场景 | 雷达水平参考方向初值 | 相机中心视线初值 | 用途 |
| --- | ---: | ---: | --- |
| 室内前向、墙面 | 下俯约 20° | 近似水平 | 画面与雷达覆盖墙面和部分地面 |
| 前下方障碍与地面 | 下俯约 30° | 下俯约 10° | 两者相对角仍约 20° |

以上只是有利于视场重叠的起点，角度必须按实际支架、雷达坐标系和目标场景
调整。尤其在四旋翼上，先用实拍和点云确认机体没有遮挡；安装角改变后要
重新做相机到雷达的外参标定。

把两者锁在同一块刚性板上。相机用机身安装孔/合适支架固定；不要靠胶带、
魔术贴或线缆维持相对位置。相机与雷达靠近但不能互相挡住视场；线缆做应力
释放，整套组件可一起隔振，不能让相机相对雷达晃动。先在地面/手持刚性组件
验证，再考虑装到四旋翼上。记录安装照片、相机面朝向、雷达顶部朝向和大致间距。
两者中心间距在不遮挡扫描的条件下尽量小；精确间距交给外参标定测量。
标定目标是 `camera_color_optical_frame`，其 `+Z` 朝前、`+X` 朝图像右、
`+Y` 朝图像下，不要把相机外壳坐标系直接当作光学坐标系。

## 2. 固定后检查驱动与相机内参

以下命令在 **不同终端**运行。本标定工具要求 Livox 驱动发布 `PointCloud2`；
原 FAST-LIVO2 建图使用 `CustomMsg`。同一雷达不能同时启动两份驱动。

先在原雷达驱动终端按 `Ctrl+C`，确认没有残留雷达发布者。然后在终端 A：

```bash
source /opt/ros/humble/setup.bash
source /home/hzw/livox_ws/install/setup.bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/install/setup.bash
ros2 launch fast_livo calibration_mid360s_points.launch.py
```

终端 B：若 D435i 相机驱动已经在运行，只检查话题，不要再启动一份。若没有，运行：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=false enable_sync:=false \
  align_depth.enable:=false enable_color:=true enable_depth:=false \
  enable_infra:=false enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=640x480x30
```

终端 C：

```bash
source /opt/ros/humble/setup.bash
ping -c 2 192.168.1.114
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic echo /livox/lidar --once --field header.stamp
ros2 topic echo /camera/camera/color/image_raw --once --field header.stamp
```

期望 `/livox/lidar` **只有 1 个发布者且类型是** `sensor_msgs/msg/PointCloud2`，
大约 10 Hz；IMU 约 200 Hz；图像约 30 Hz。CameraInfo 应为 `640×480`、
`plumb_bob`，`K` 约为 `[608.162,0,323.260; 0,607.041,251.738; 0,0,1]`，
`D` 全零。每次改 RGB 分辨率、裁切或更换相机都重新核对并更新内参文件。
两类消息时间戳应处于同一时间尺度且单调；这项检查不能证明曝光时间同步。
`ros2 topic hz` 会持续统计，读到频率后按 `Ctrl+C` 再执行下一条。

## 3. 录制 5 组静止标定 bag

在工程根目录的终端 C：

```bash
source /opt/ros/humble/setup.bash
cd /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics
CALIB_ROOT="$(pwd)/calibration_data"
mkdir -p "$CALIB_ROOT/bags" "$CALIB_ROOT/preprocessed"
ros2 bag record -o "$CALIB_ROOT/bags/pose_01" \
  /livox/lidar /livox/imu \
  /camera/camera/color/image_raw /camera/camera/color/camera_info
```

开始录制前先让装置静止，保持 **10～15 秒**后按 `Ctrl+C` 正常结束。检查：

```bash
ros2 bag info "$CALIB_ROOT/bags/pose_01"
```

换一个朝向或位置，把 `pose_01` 分别改为 `pose_02`～`pose_05`，每组都先静止
再录 10～15 秒。场景选 1～5 m 的墙角、门框、桌椅、招牌等具有几何与反射纹理
的物体，避免整幅纯白墙、玻璃、近距离黑色物体、剧烈曝光或移动人群。
每组 bag 要同时有图像、CameraInfo、`PointCloud2`；相机和雷达在各组之间
始终保持固定。官方工具对 Livox 建议静止积累 10～15 秒，并推荐 5～10 组。

## 4. 用预编译 Docker 标定，避免在本机编译大型依赖

本机已安装 Docker，但标定镜像尚未下载。以下操作在刚性安装且 bag 录好后进行。
镜像首次下载需要网络和磁盘；标定容器限制为 2 个 CPU、5 GiB 内存，以免影响
桌面系统。先仅预处理；若内存上限导致容器退出，减少输入 bag 组数再试。

```bash
cd /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics
CALIB_ROOT="$(pwd)/calibration_data"
docker pull koide3/direct_visual_lidar_calibration:humble
docker run --rm --cpus=2 --memory=5g --memory-swap=5g \
  -v "$CALIB_ROOT/bags:/input:ro" \
  -v "$CALIB_ROOT/preprocessed:/output" \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess -a \
  --min_distance 0.5 /input /output
ls "$CALIB_ROOT/preprocessed"
```

预处理目录应有各组的图像、点云文件和 `calib.json`。如果出现找不到图像或点云，
先用 `ros2 bag info` 确认录制的话题类型和数量。**不要用 CustomMsg bag 直接
喂给这个预处理器**，它要求 `PointCloud2`。

当前机器是 X11；交互工具可用下面的 shell 函数。在同一个终端定义：

```bash
calib_gui() {
  docker run -it --rm --cpus=2 --memory=5g --memory-swap=5g \
    -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/calib_xauthority \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$XAUTHORITY:/tmp/calib_xauthority:ro" \
    -v "$CALIB_ROOT/preprocessed:/data" \
    koide3/direct_visual_lidar_calibration:humble \
    ros2 run direct_visual_lidar_calibration "$1" /data
}
calib_gui initial_guess_manual
```

在 GUI 里从点云和图像选至少 3 对对应点，尽量覆盖不同深度和画面位置；
依次点 `Add picked points`、`Estimate`、检查投影、`Save`。再运行：

```bash
calib_gui calibrate
calib_gui viewer
```

在 `viewer` 里检查至少几组不同方位、近远不同的墙角/物体边缘是否贴合。
如果只有一组贴合而其他组明显错位，重新检查安装是否松动、点对选取、
相机内参和场景重叠。`preprocessed/calib.json` 的
`results.T_lidar_camera = [tx,ty,tz,qx,qy,qz,qw]` 是**相机光学系到雷达**，
不能原样填进 FAST-LIVO2 的 `Rcl/Pcl`。

## 5. 转换外参并回到 FAST-LIVO2

在工程根目录运行（若本机软件源缺 `numpy/scipy/yaml`，先检查对应包；当前机器
这三项已经存在）：

```bash
cd /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics
CALIB_ROOT="$(pwd)/calibration_data"
python3 scripts/apply_camera_lidar_calibration.py \
  "$CALIB_ROOT/preprocessed/calib.json" \
  src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml
```

脚本按 `Rcl = R_lidar_camera^T`、`Pcl = -Rcl·t_lidar_camera` 求逆，写出
`img_en: 1` 的独立配置，并检查内参是否与当前 D435i 的 `640×480` 设置匹配。
先在标定 viewer 检查重投影；绝不能因为脚本生成了 YAML 就认为标定正确。

在终端 A 停止标定专用驱动，再启动原本的 **CustomMsg** 驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/hzw/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

新终端核对 `/livox/lidar` 为 `livox_ros_driver2/msg/CustomMsg` 且发布者数为 1，
然后启动融合建图（不要同时启动 FAST-LIO2；先不启 RViz）：

```bash
cd /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics
source /opt/ros/humble/setup.bash
source /home/hzw/livox_ws/install/setup.bash
source install/setup.bash
OMP_NUM_THREADS=1 nice -n 15 ros2 launch fast_livo mapping_mid360s_d435i.launch.py \
  visual_config_file:="$(realpath src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml)" \
  use_rviz:=false
```

## 6. 验证融合与时间对齐

另一个终端检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /aft_mapped_to_init
ros2 topic info /camera/camera/color/image_raw
ros2 topic echo /cloud_registered --once --field width
ros2 topic echo /cloud_registered --once --field fields
```

验收次序：①三路输入稳定；②融合节点有图像订阅者，里程计/点云持续发布；
③运行日志中出现 VIO 处理，视觉地图**有非零新增点**；④标定 viewer 与实际
画面中的近远物体投影一致；⑤缓慢平移、转动整套刚性组件，轨迹连续，墙面不
明显重影。仅看到 VIO 循环或里程计频率高于 10 Hz 不足以证明视觉融合成功。

`img_time_offset` 当前从 `0.0 s` 起步。静止标定不能估计运动时的曝光/雷达
时间差；本工程 `soft-sync` 的 IMU 修正也不是三传感器硬件同步证明。若运动时
投影拖尾、墙面重影，先检查时间戳与掉帧，再对着刚性场景缓慢往返旋转，
在同一份数据上小步试验图像时间偏移，并以投影残差和建图稳定性决定取值。
不要照搬 Avia 示例的 `0.1 s`。飞行前完成地面测试。

## 7. 执行勾选表

- [ ] 相机和雷达固定在同一刚性板上，支架和线缆不松动。
- [ ] RGB 图像与点云存在明显共同视场，机体/支架没有大面积遮挡。
- [ ] `PointCloud2` 标定驱动独占雷达，三种数据频率及内参核对完成。
- [ ] 录制至少 5 组各 10～15 秒的静止 bag，`ros2 bag info` 有完整消息。
- [ ] 预处理、人工初值、精配准及多组重投影检查完成。
- [ ] 把 `T_lidar_camera` **求逆**得到 `Rcl/Pcl`，生成独立 YAML。
- [ ] 切回 `CustomMsg` 驱动，确认没有重复雷达发布者。
- [ ] FAST-LIVO2 的 VIO 地图新增点不再长期为零，近远场投影和运动建图均合理。
- [ ] 对运动数据检查时间偏移，地面测试稳定后再上机飞行。

## 参考依据

- [Livox Mid-360S 官方规格](https://www.livoxtech.com/mid-360s/specs)
- [Livox Mid-360S 官方手册下载](https://www.livoxtech.com/mid-360s/downloads)
- [ROS REP-103 光学坐标系约定](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst)
- [Koide 标定工具数据采集要求](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
- [Koide 标定工具程序参数与外参方向](https://koide3.github.io/direct_visual_lidar_calibration/programs/)
- [Koide 官方 Docker 命令](https://koide3.github.io/direct_visual_lidar_calibration/docker/)
