> **当前完整教程入口：** [标定 README](../../docs/CALIBRATION_README.md) · [FAST-LIVO2 调试 README](../../docs/FASTLIVO2_DEBUG_README.md) · [13.42 cm 存档](../../calibration_archives/2026-10-17_mid360s_d435i_13p42cm/README.md)。下文保留历史笔记，遇到冲突以新手册为准。

你现在这个现象正常：**`ping` 通说明雷达网络通了，ROS 节点要启动驱动后才会出现。** 我检查到驱动包、配置文件和相机都在，但当前没有雷达、相机驱动进程运行。

`source` 只加载环境，`ros2 node list` 只查询节点，它们都不会启动驱动。下面按顺序操作，先打开 **A、B、C 三个终端**。

**1. 终端 A：启动雷达标定驱动**

先关闭 Viewer2，以及此前运行的 FAST-LIO2、FAST-LIVO2、Livox 驱动。然后复制：

```bash
# 加载 ROS 2、Livox 驱动、当前 FAST-LIVO2 工程的环境
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh

# 三个终端使用相同的 ROS 通信域，确保能相互发现节点
export ROS_DOMAIN_ID=0

# 启动雷达驱动
# 这个启动文件发布 PointCloud2，供标定工具使用
ros2 launch fast_livo calibration_mid360s_points.launch.py
```

**保持这个终端运行。** 启动后没有返回命令提示符通常是正常的，说明驱动在前台运行。

预期日志包含驱动初始化、雷达连接、点云和 IMU 发布信息。如果出现 `process has died` 或初始化失败，先处理该错误，再继续。

**2. 终端 B：启动 D435i 彩色相机**

```bash
# 加载同一套环境
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0

# 只启用 RGB，使用当前工程已有的 640×480 相机参数
# 深度、红外和相机 IMU 暂时关闭，降低负载
# 当前 FAST-LIVO2 使用的是雷达内置 IMU
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  serial_no:=_261722072542 \
  enable_color:=true \
  rgb_camera.color_profile:=640x480x30 \
  enable_depth:=false \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  enable_rgbd:=false \
  enable_sync:=false \
  align_depth.enable:=false
```

序列号前面的 `_` 是驱动对纯数字序列号的字符串写法。

**这个终端也保持运行。** 日志应显示相机识别和 Color 流启动。

**3. 终端 C：现在再查节点和话题**

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0

# 直接发现节点，暂时绕过 ROS CLI 的后台缓存
ros2 node list --no-daemon --spin-time 3

# 查看驱动发布了哪些话题
ros2 topic list
```

节点列表预期至少包含：

```text
/livox_lidar_publisher
/camera/camera
```

话题列表预期包含：

```text
/livox/lidar
/livox/imu
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

如果仍看不到节点：

```bash
# 重启 ROS 命令行的发现服务；这不会重启或停止传感器驱动
ros2 daemon stop
ros2 daemon start

# 再查询一次
ros2 node list --no-daemon --spin-time 5
```

如果还是空的，检查 A、B 两个终端是否已经报错退出。**此时先不要录包，需要看驱动终端的报错。**

节点出现后，继续检查数据：

```bash
# 雷达必须是 sensor_msgs/msg/PointCloud2，发布者数量应为 1
ros2 topic info /livox/lidar

# 图像必须是 sensor_msgs/msg/Image，发布者数量应为 1
ros2 topic info /camera/camera/color/image_raw

# 每条命令统计约 8 秒后自动停止
timeout --signal=INT 8s ros2 topic hz /livox/lidar
timeout --signal=INT 8s ros2 topic hz /livox/imu
timeout --signal=INT 8s ros2 topic hz /camera/camera/color/image_raw
```

预期频率：

| 数据 | 预期 |
|---|---:|
| 雷达点云 | 约 10 Hz |
| 雷达 IMU | 约 200 Hz |
| RGB 图像 | 约 30 Hz |

`timeout` 到时结束可能返回状态码 124，这是计时结束，不代表驱动故障。

**4. 保存相机内参，检查画面**

仍在终端 C：

```bash
# CALIB_ROOT 已由环境脚本定义
# 本次目录：calibration_data/rigid_mount_01
mkdir -p "$CALIB_ROOT/bags" "$CALIB_ROOT/preprocessed"

# 显示并保存当前 RGB 相机内参
ros2 topic echo /camera/camera/color/camera_info --once \
  | tee "$CALIB_ROOT/camera_info.yaml"

# 确认点云字段包含 x、y、z、intensity
ros2 topic echo /livox/lidar --once --field fields

# 打开图像预览
ros2 run rqt_image_view rqt_image_view
```

在窗口中选择 `/camera/camera/color/image_raw`。确认图像清晰、不过曝，能看到墙角、门框、箱子等结构。

CameraInfo 应为 `640×480`。当前工程保存的内参约为：

```text
fx = 608.162
fy = 607.041
cx = 323.260
cy = 251.738
```

本轮先使用出厂 CameraInfo。若当前内参与上述值明显不同，先核对相机和分辨率，避免标定工具与 FAST-LIVO2 使用两套模型。

检查完关闭预览窗口，减轻录制负载。

**5. 录制至少 5 组标定数据**

每组都要遵守：

- 相机和雷达始终保持刚性固定。
- 每组开始前，把**整套组件**放稳。
- 录制期间完全静止。
- 组与组之间改变整套组件的位置、朝向和观察场景。
- 共同视野内尽量有不同距离的物体、几何边缘和反射纹理。

Livox 非重复扫描需要静止积累点云，官方建议每组 10～15 秒、采集 5～10 组；下面录制 20 秒，留出发现话题的时间。[采集要求](https://koide3.github.io/direct_visual_lidar_calibration/collection/)

在终端 C 复制以下函数：

```bash
# 定义一个录制函数，不会立即开始录制
record_pose() {
  local pose_name="$1"

  # 防止名称为空或覆盖已有数据
  if [ -z "$pose_name" ] || [ -e "$CALIB_ROOT/bags/$pose_name" ]; then
    echo "请输入新名称，不能覆盖已有 bag"
    return 1
  fi

  # 录制约 20 秒，到时用 SIGINT 正常结束
  # sqlite3 是当前标定流程使用的 bag 存储格式
  # 缓存限制为 32 MiB，减少内存占用
  timeout --signal=INT --kill-after=10s 20s \
    ros2 bag record \
      --storage sqlite3 \
      --max-cache-size 33554432 \
      -o "$CALIB_ROOT/bags/$pose_name" \
      /livox/lidar \
      /livox/imu \
      /camera/camera/color/image_raw \
      /camera/camera/color/camera_info

  # 录完检查实际话题、消息数量和时长
  ros2 bag info "$CALIB_ROOT/bags/$pose_name"
}
```

放稳组件，然后录第一组：

```bash
record_pose pose_01
```

换一个位置、朝向，放稳后录第二组：

```bash
record_pose pose_02
```

再次改变观察视角，放稳后分别执行：

```bash
record_pose pose_03
```

```bash
record_pose pose_04
```

```bash
record_pose pose_05
```

**不要一次连续执行五次调用而不改变视角。** 每个 bag 应包含四个话题，点云约百余帧、图像数百帧。

全部录完后，在终端 A、B 分别按 `Ctrl+C` 停止驱动，再进行离线标定。

**6. 下载预编译工具并预处理**

在终端 C：

```bash
# 若换了新终端，先重新加载环境
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh

# 下载官方 Humble 镜像，不在本机编译大型依赖
docker pull "$CALIB_IMAGE"

# 保存本次工具镜像的标识，方便以后追溯版本
docker image inspect "$CALIB_IMAGE" --format '{{.Id}}' \
  > "$CALIB_ROOT/solver_image_id.txt"

# 预处理：从 bag 提取图像、内参，并积累静止点云
# 限制为 1 个 CPU、5 GiB 内存，减少对桌面的影响
docker run --rm \
  --cpus=1 --memory=5g --memory-swap=5g \
  -e OMP_NUM_THREADS=1 \
  -e OPENBLAS_NUM_THREADS=1 \
  -v "$CALIB_ROOT/bags:/input:ro" \
  -v "$CALIB_ROOT/preprocessed:/output" \
  "$CALIB_IMAGE" \
  ros2 run direct_visual_lidar_calibration preprocess /input /output \
    --image_topic /camera/camera/color/image_raw \
    --camera_info_topic /camera/camera/color/camera_info \
    --points_topic /livox/lidar \
    --intensity_channel intensity \
    --min_distance 0.5

# 查看生成的文件
ls -lh "$CALIB_ROOT/preprocessed"
```

应出现 `calib.json` 和各场景的图像、点云文件。若预处理报错或结果只有空目录，先解决再继续。

**7. 准备标定窗口的显示权限**

仍在终端 C：

```bash
# 为 Docker 创建本次桌面会话的 X11 授权文件
export CALIB_XAUTH="$CALIB_ROOT/docker.xauth"
touch "$CALIB_XAUTH"
chmod 600 "$CALIB_XAUTH"

# 复制当前显示会话的授权，允许容器显示窗口
xauth nlist "$DISPLAY" \
  | sed 's/^..../ffff/' \
  | xauth -f "$CALIB_XAUTH" nmerge -

# 文件非空才说明已经获得授权
test -s "$CALIB_XAUTH" && echo "X11 授权文件已准备"
```

如果没有出现“X11 授权文件已准备”，先不要启动 GUI。

接着定义 GUI 命令：

```bash
# 定义统一的容器启动函数
# 后面可以用 calib_gui initial_guess_manual / calibrate / viewer
calib_gui() {
  docker run --rm -it \
    --network host --ipc host \
    --cpus=1 --memory=5g --memory-swap=5g \
    -e OMP_NUM_THREADS=1 \
    -e OPENBLAS_NUM_THREADS=1 \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/calib.xauth \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$CALIB_XAUTH:/tmp/calib.xauth:ro" \
    -v "$CALIB_ROOT/preprocessed:/data" \
    "$CALIB_IMAGE" \
    ros2 run direct_visual_lidar_calibration "$@" /data
}

# 打开人工初值窗口
calib_gui initial_guess_manual
```

**8. 人工选点、优化、检查投影**

人工初值窗口操作：

1. 右键选点云中一个明显角点，再右键选图像中的同一个物理点。
2. 点击 `Add picked points` 保存这一对对应点。
3. 重复选点，建议选 6～10 对，分布在不同位置和深度。
4. 点击 `Estimate`。
5. 调整 `blend_weight` 检查点云和图像的大致对应关系。
6. 点击 **`Save`**，然后关闭窗口。

工具最低要求是三对点；选点分散、对应准确，更有利于得到可靠初值。[人工初值说明](https://koide3.github.io/direct_visual_lidar_calibration/programs/)

回到终端 C：

```bash
# 从人工初值开始精标定；结束后自动退出
calib_gui calibrate --auto_quit

# 打开结果检查窗口
calib_gui viewer
```

逐组观察：门框、墙角和箱子边缘应贴合；近处和远处都不能明显错位。**不能只看一个场景好看，就认为标定完成。**

**9. 转换成 FAST-LIVO2 外参**

Koide 输出的是“相机光学系 → 雷达”；FAST-LIVO2 需要“雷达 → 相机光学系”。现有脚本会自动求逆：

```bash
cd "$LIVO_WS"

# 读取实测结果，生成新的视觉融合配置
# 不会覆盖已有同名文件
python3 scripts/apply_camera_lidar_calibration.py \
  "$CALIB_ROOT/preprocessed/calib.json" \
  src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml
```

如果脚本提示内参或畸变不一致，先统一相机模型，不要删除检查绕过。生成 YAML 说明转换成功，标定质量仍要靠上一步的投影检查确认。

**10. 切回建图驱动，启动 FAST-LIVO2**

终端 A：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0

# 建图使用 CustomMsg，区别于前面标定使用的 PointCloud2
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

终端 B：重新执行步骤 2 的 D435i RGB 启动命令。

终端 C：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0

# 此时应显示 livox_ros_driver2/msg/CustomMsg，发布者为 1
ros2 topic info /livox/lidar

cd "$LIVO_WS"

# 使用刚生成的实测外参配置启动融合
# 先关闭 RViz，并限制 OpenMP 线程，降低负载
OMP_NUM_THREADS=1 nice -n 15 \
  ros2 launch fast_livo mapping_mid360s_d435i.launch.py \
    visual_config_file:="$LIVO_WS/src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml" \
    use_rviz:=false
```

启动后先静置几秒，再缓慢移动整套组件。后续要确认视觉新增点非零、轨迹连续、墙面没有明显重影。静止外参标定不能测出时间偏移，RGB 时偏还需要运动数据验证。

**你现在先执行步骤 1～3。** 预期会看到雷达和相机两个节点；如果没有，保留并查看终端 A、B 的完整启动报错，暂时不要进入录包阶段。