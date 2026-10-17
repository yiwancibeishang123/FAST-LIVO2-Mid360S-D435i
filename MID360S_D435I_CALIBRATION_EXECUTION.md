> **历史说明，包含当时的旧数据/旧初值状态。** 当前逐步操作统一看 [标定手册](docs/CALIBRATION_README.md)；运行融合看 [调试手册](docs/FASTLIVO2_DEBUG_README.md)。

# 已刚性固定的 D435i + Mid-360S：标定执行步骤

更新：2026-10-17。设备已由用户确认刚性固定；本机检查到 D435i USB 设备，
`eno1=192.168.1.50/24`，雷达 `192.168.1.114` ping 成功。本文件是操作说明；
尚未采集本次安装的标定数据，也没有生成实测外参。

目标是 `p_color = Rcl * p_lidar + Pcl`，其中 color 是 RGB 光学坐标系
（x 向图像右、y 向下、z 向前），Pcl 单位为米。
当前 FAST-LIVO2 使用 Livox 内置 IMU，不使用 D435i IMU 或深度图。
先核对相机内参，再求相机–雷达外参；时间偏移需另外用运动数据验证。

## 为什么图像预览只有一个视角

当前 FAST-LIVO2 前端订阅一条 `common.img_topic`（本机为 RGB image_raw），
用雷达地图提供三维几何，再用图像纹理约束运动。它不要求双目视差提供深度。
D435i 的左右红外相机主要用于自身的双目深度；本轮没有启用它们或深度流。
因此看到一幅 RGB 图像是正确现象；这里标定的是 RGB 光学系与雷达的相对位姿，
不是左右红外相机的双目标定。使用相机 IMU 的其他算法会有另外的标定链。

当前采用 Koide 无靶标方法，不运行四圆/AprilTag 检测器，也不需要输入标定板尺寸。
有标记的板可作为场景纹理，但不能五组都只录同一个平面：让门框、墙角、箱子等
不同距离结构进入共同视场。画面中的强反光、镜面倒影和过曝区域尽量避开。
仅做出厂内参核对不等于重新完成了一次 RGB 内参标定。

## 1. 准备终端和数据目录

每个新终端都先粘贴下面这一行。它只加载已有环境并定义目录，不启动传感器：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
```

本次数据根目录为 `$CALIB_ROOT`，即 `calibration_data/rigid_mount_01`。
拆装、松动、调整相机/雷达后，要在上述脚本改用新的目录名，并重新标定。
同一次安装可以用 pose_06、pose_07 继续补采，不覆盖已有 bag。

```bash
mkdir -p "$CALIB_ROOT/bags" "$CALIB_ROOT/preprocessed"
ip -br addr show eno1
ping -c 3 -W 1 192.168.1.114
ros2 node list
```

此时若还没启动任何驱动，`ros2 node list` 没有输出是正常现象；ping 成功只说明
雷达网络可达，不会自动创建 ROS 节点。直接继续步骤 2、3 启动驱动，保留它们的
终端运行，再在终端 C 查询节点。预期能看到 `/livox_lidar_publisher` 和
`/camera/camera`。`source` 仅加载软件环境，也不会启动节点。

关闭 Viewer2、FAST-LIO2、FAST-LIVO2 和旧雷达驱动；在它们各自的终端按 Ctrl+C。
同一雷达只启动一份驱动。离线处理时也关闭相机/雷达驱动和图像预览，减少负载。

## 2. 终端 A：启动标定用雷达驱动

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
ros2 launch fast_livo calibration_mid360s_points.launch.py
```

此启动文件设置 `xfer_format=0`，发布 `sensor_msgs/msg/PointCloud2`。
保留驱动内置外参为零，避免驱动先转一次坐标、FAST-LIVO2 再转一次。

## 3. 终端 B：只启动 D435i RGB

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  serial_no:=_261722072542 \
  enable_color:=true rgb_camera.color_profile:=640x480x30 \
  enable_depth:=false enable_infra:=false enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  enable_rgbd:=false enable_sync:=false align_depth.enable:=false
```

序列号前的下划线是 RealSense ROS 驱动对纯数字序列号的字符串写法。
整个标定和首次融合测试保持同一 RGB 分辨率、图像话题和相机模型。

## 4. 终端 C：核对数据、内参和画面

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
ros2 topic info /livox/lidar --verbose
ros2 topic info /camera/camera/color/image_raw --verbose
timeout --signal=INT 8s ros2 topic hz /livox/lidar
timeout --signal=INT 8s ros2 topic hz /livox/imu
timeout --signal=INT 8s ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once | tee "$CALIB_ROOT/camera_info.yaml"
ros2 topic echo /livox/lidar --once --field fields
```

`timeout` 到时停止频率统计，返回 124 是正常到时退出，不代表驱动故障。
雷达应为 PointCloud2、约 10 Hz；图像为 Image、约 30 Hz；IMU 约 200 Hz。
两个主要数据话题各有一个发布者；雷达字段包含 x/y/z/intensity。
CameraInfo 应为 640×480、plumb_bob；当前保存的相机参数为
fx=608.162、fy=607.041、cx=323.260、cy=251.738，D 为零。
应以本次 CameraInfo 为准，差异明显时先核对相机和分辨率，再更新相机 YAML。

第一轮用出厂 CameraInfo 内参完成外参标定。D 为零是驱动提供的参数，不能据此
宣称已验证真实镜头无畸变；若边缘投影持续呈系统性弯曲或需要更高精度，再独立
进行 RGB 棋盘格/AprilGrid 内参标定，并让标定工具和 FAST-LIVO2 使用同一个模型。
下面的现有转换脚本会拒绝与保存内参不一致或非零畸变的结果，避免悄悄混用模型。

图像预览（在另一个已加载环境的终端；录包前关闭以减轻负载）：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/camera/camera/color/image_raw`。确认墙角、门框、箱子等结构清晰，画面不过曝。

## 5. 采集至少 5 组静止场景

无须专用标定板。选有几何边缘和不同反射纹理的真实场景，避免纯白墙、玻璃和
大量运动物体。近处约 1–2 m 与远处约 3–5 m 的结构都应出现在共同视野中。
每次只移动整个刚性组件，放稳后再录；录制全程不动。场景之间改变观察位置、
方向和深度分布，不能五次都录同一角度。该静止采集要求来自 Koide 官方文档。

在终端 C 定义以下函数，每次调用自动录制约 20 秒（含话题发现时间），
正常发 SIGINT 结束并打印 bag 信息。这里不用 shell 的 set -e。

```bash
record_pose() {
  local pose_name="$1"
  if [ -z "$pose_name" ] || [ -e "$CALIB_ROOT/bags/$pose_name" ]; then
    echo "需要新名称，且不能覆盖已有 bag"
    return 1
  fi
  timeout --signal=INT --kill-after=10s 20s \
    ros2 bag record --storage sqlite3 --max-cache-size 33554432 \
      -o "$CALIB_ROOT/bags/$pose_name" \
      /livox/lidar /livox/imu \
      /camera/camera/color/image_raw /camera/camera/color/camera_info
  local record_status=$?
  if [ "$record_status" -ne 0 ] && [ "$record_status" -ne 124 ]; then
    echo "录制异常退出：$record_status，请检查上面的日志并保留现场数据"
    return "$record_status"
  fi
  ros2 bag info "$CALIB_ROOT/bags/$pose_name"
}

record_pose pose_01
```

放稳后分别执行下一条；不要一次粘贴完四次调用而忘记改变位置：

```bash
record_pose pose_02
```

```bash
record_pose pose_03
```

```bash
record_pose pose_04
```

```bash
record_pose pose_05
```

每包应有四个话题，持续十几秒以上、点云约百余帧、图像数百帧，无明显录制丢包。
raw RGB 约 28 MB/s；20 秒一包量级为数百 MB，实际看 bag info/du。
完成后在终端 A、B 按 Ctrl+C 停止驱动。

## 6. 拉取预编译工具并预处理

不编译大型依赖。容器限制 1 个 CPU、5 GiB 内存，线程数设为 1；
这降低负载，但不构成系统绝不会卡顿的保证。
本机 Docker 可用，Koide Humble 镜像在本次检查时还没有下载。

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
docker pull "$CALIB_IMAGE"

# 检查本轮五组数据；若录成 CustomMsg，先离线转换为标定用 PointCloud2
# 使用空目录预处理，验证成功后归档旧结果并替换当前 preprocessed
cd "$LIVO_WS"
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 15 \
  python3 scripts/preprocess_calibration.py

ls -lh "$CALIB_ROOT/preprocessed"
python3 -m json.tool "$CALIB_ROOT/preprocessed/calib.json"
```

脚本显式指定话题，使用 bag 中的 CameraInfo。静止 Livox 数据不用 `-d` 动态积分。
预处理应生成 calib.json 和各场景非空的图像、点云文件；只有空目录或异常时先停下排查。
脚本每次使用新的临时输出目录，成功后将旧结果归档为 preprocessed_archive_*。
旧的 mid360s_d435i_calibrated.yaml 同时归档为 old_visual_config_*.yaml，
需完成本轮初值和精标定后重新生成配置。出现错误时保留旧预处理目录。
重新采集后不能直接使用旧标定窗口：关闭旧 GUI 后重新打开 initial_guess_manual。

2026-10-17 本轮重采 bag 为 CustomMsg，已离线转换到 bags_points_01；
原始 bags 未改动。五组图像与新原始 bag 的首帧处理结果逐像素一致，点云首帧
XYZ/intensity、header、bag 时间戳核对一致，各话题消息数保持不变。
PointCloud2 转换只用于静止标定，省略逐点时间、tag 和 line；工具会提示逐点时间
缺失，此处未进行运动去畸变。FAST-LIVO2 建图仍使用原始 CustomMsg 驱动。

## 7. 生成 X11 授权，并定义 GUI 命令

在桌面会话的终端 C 执行，保持该终端供 initial/calibrate/viewer 使用。
本机使用 X11，DISPLAY=:1；读取当前变量，无须 xhost +。

```bash
export CALIB_XAUTH="$CALIB_ROOT/docker.xauth"
touch "$CALIB_XAUTH"
chmod 600 "$CALIB_XAUTH"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$CALIB_XAUTH" nmerge -
test -s "$CALIB_XAUTH" && echo "X11 授权文件已准备"

calib_gui() {
  docker run --rm -it --network host --ipc host \
    --cpus=1 --memory=5g --memory-swap=5g \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/calib.xauth \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$CALIB_XAUTH:/tmp/calib.xauth:ro" \
    -v "$CALIB_ROOT/preprocessed:/data" \
    "$CALIB_IMAGE" \
    ros2 run direct_visual_lidar_calibration "$@" /data
}

calib_gui initial_guess_manual
```

若 X11 授权文件为空，不继续启动 GUI；检查 DISPLAY、XAUTHORITY 和 xauth list，
不要粘贴 cookie 到聊天。docker.xauth 含会话授权，不加入 Git。

## 8. 人工初值、精标定、独立检查

在 initial_guess_manual 窗口中：

1. 右键选点云中一个可确定的角点，再右键选图像中的同一物理点。
2. 点 Add picked points，重复选点。工具最少需要 3 对，实际建议 6–10 对，
   分布在不同位置和深度，避免所有点共线或集中在一小块平面。
3. 点 Estimate，用 blend_weight 调整融合显示，确认整体投影方向合理。
4. 点 Save 后关闭窗口；只关窗口而未 Save 不会保存初值。

五组共用同一套相机–雷达外参，不需要为每组分别估计并保存一套外参。
在一组中找到可靠对应点即可提供初值；也可用 bag_id 左右箭头切换组，继续加入
各组内正确的 2D–3D 点对，然后统一 Estimate / Save。每一对必须来自同一组场景。
切换 bag_id 不清除已加入的点对。精标定默认使用 calib.json 列出的全部五组，
不要加 `--first_n_bags 1`。每组只检查投影，不必重新选点。

初值质量先看投影及基线是否合理：T_lidar_camera 的平移表示两个传感器坐标
原点间的位移，不是到墙面的距离。2026-10-17 本次用户初值平移为
[2.91117, −0.98818, 0.77285] m，长度约 3.17 m，与同一支架不符，需重选初值。
原文件已备份到 calibration_data/rigid_mount_01/calib.initial_suspect_backup_01.json。
当前只存在 init_T_lidar_camera，不能当作精标定结果导入。
清空旧对应点的可靠方法是关闭并重启 initial_guess_manual；窗口左下的 clear
仅清空日志，不能用于删除已加入的对应点。重启后保持 0 deg，选不同距离的真实
几何角点，避免错误匹配、反光和误拖平移/旋转控制柄。Save 会替换人工初值。

同一终端依次执行：

```bash
calib_gui calibrate --auto_quit
calib_gui viewer
```

优化结束自动写入结果；viewer 中逐组检查门框、墙角、箱子边缘，检查近远物体和
图像上下左右区域。最好补采一两组未参与求解的数据做独立投影检查；用于评估的
数据不应再混入用于拟合的 bags 目录。一组好看不能代替多组几何检查。

检验清单：图像/点云边缘整体一致；近远距离不系统错位；重复标定结果稳定；
估计中心间距与实测支架尺寸同量级；不出现明显镜像、180°反向或几十米平移。
这些是工程验收项，不是未经测量即可宣称的毫米精度。

## 9. 求逆转换成 FAST-LIVO2 参数

Koide 输出 `T_lidar_camera`，即相机光学系→雷达；FAST-LIVO2 要雷达→相机。
现有脚本自动求逆，写独立 YAML，不覆盖已有文件。

```bash
cd "$LIVO_WS"
python3 scripts/apply_camera_lidar_calibration.py \
  "$CALIB_ROOT/preprocessed/calib.json" \
  src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml
```

如输出文件已存在，换新名称保存本轮结果。若脚本报告内参/畸变不一致，先统一
CameraInfo、标定相机模型和 camera_d435i_261722072542.yaml，不能直接删除检查绕过。
该脚本仅转相机–雷达外参，雷达–IMU 的 extrinsic_T/R 保持模板值，需单独核实方向。
本机模板 `p_imu = R * p_lidar + T`，T=[−0.011,−0.02329,+0.04412] m；
若资料给的是 `p_lidar = T_lidar_imu * p_imu`，需要按变换方向求逆，不能仅复制数字。

## 10. 切回 CustomMsg，并验证视觉融合

终端 A：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

终端 B：重新执行步骤 3 的 RGB 启动命令。

终端 C：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
ros2 topic info /livox/lidar
cd "$LIVO_WS"
OMP_NUM_THREADS=1 nice -n 15 ros2 launch fast_livo mapping_mid360s_d435i.launch.py \
  visual_config_file:="$LIVO_WS/src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml" \
  use_rviz:=false
```

雷达现在应为 `livox_ros_driver2/msg/CustomMsg` 且只有一个发布者。
保持组件静止几秒完成初始化，再缓慢平移、转动整个支架，检查 VIO 新增点非零、
位姿连续、墙面无明显重影。`img_time_offset=0` 只是初值，静止数据无法估计时偏。
本工程 `LIVMapper.cpp` 对图像采用 `raw_img_time - img_time_offset`，时偏必须按这个
符号定义和实际 RGB 时钟标定，不能复制其他设备的深度时偏。
具体地，若本机测得 `t_RGB = t_Livox + delta`，应设 `img_time_offset = delta`，
使 `t_RGB_corrected = t_RGB - delta`。这是本工程的定义，不能从另一个实现的配置照搬符号。

## 融合自行停止的诊断

若融合在无人操作时退出，终端 C 可改用诊断入口（终端 A、B 驱动照常运行）：

```bash
bash /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/run_fusion_diagnostics.sh
```

该入口不限制运行时长，记录完整屏幕输出与停止信号发送 PID。
2026-10-17 已修复 native 编译参数导致的 Eigen/PCL 内存释放不兼容，
并将算法线程默认限制为 1。离线回放加等待超过 60 秒、主动停止正常退出验证通过。
历史未知 SIGINT 的来源仍需用新诊断日志确认。详见工程根目录
`FASTLIVO2_SHUTDOWN_DIAGNOSIS.md`。

## 你提供的参考仓库怎么用

已审阅 `KaiFeng-Frank/lidar-camera-imu-calibration` 的雷视工作流、入口脚本和
下游验证报告。其雷视 solver 也采用 Koide，可参考静止多场景、点对初值、
数据/安装会话记录、投影复核及多轮标定的一致性检查。

原入口按 Jazzy、1280×720、自标 Kalibr RGB 内参及其设备身份配置；当前工程为
Humble、640×480。先按本文件执行，不能直接套其启动路径、K/D、T、IMU修正或时偏。
该仓库的可视化去畸变也不能直接叠加在 FAST-LIVO2 已有的去畸变前，否则会重复补偿。
其 FAST-LIVO2 和 Coco-LIC 验证未给出独立真值，属于功能和一致性证据。

特别核对：其 `results/fastlivo2_downstream_validation.json` 将 +5.989 ms 描述为
按 `stamp + img_time_offset` 消费的参数，并注明原时偏针对 depth，color 还可能有额外差异。
本工程却是 `stamp - img_time_offset`，所以既有流类型差异，也有消费者符号差异；
该数字不能作为本机 RGB 的配置初值。报告中的 D435i serial 为 947122070908，
本机为 261722072542，外参也绑定不同的刚性安装。

下一阶段可分别复用其 RGB 内参标定、Mid-360S 多姿态 IMU 标定、
时偏估计和独立验证方法；这些应使用本机重新采集的数据和参数。
当前 FAST-LIVO2 用雷达 IMU，D435i 自带 IMU 与深度模型并非第一轮雷视融合的前置项。

参考：
- https://koide3.github.io/direct_visual_lidar_calibration/collection/
- https://koide3.github.io/direct_visual_lidar_calibration/programs/
- https://koide3.github.io/direct_visual_lidar_calibration/docker/
- https://github.com/KaiFeng-Frank/lidar-camera-imu-calibration/blob/master/docs/LIDAR_CAMERA_EXTRINSIC.md
- https://github.com/KaiFeng-Frank/lidar-camera-imu-calibration/blob/master/results/fastlivo2_downstream_validation.json
