# Mid-360S + D435i 标定手册

[返回项目首页](../README.md) · [下一步：FAST-LIVO2 调试](FASTLIVO2_DEBUG_README.md) · [13.42 cm 存档](../calibration_archives/2026-10-17_mid360s_d435i_13p42cm/README.md)

适用：Ubuntu 22.04、ROS 2 Humble、Mid-360S、D435i RGB 640×480@30；更新 2026-10-17。
**这是从准备、录制、人工初值、精优化到导入融合的完整操作顺序。** 本机已有软件环境，正常执行不需要重新编译。

> 当前保存的是一套可追溯的候选外参：基线 13.4195 cm，机械估测约 13 cm。视觉更新已经出现，但静置融合仍有明显位姿波动，尚未完成准确性验收。外参只适用于本次刚性安装，不能复制到另一套支架。

## 导航

1. [先理解要标什么](#1-先理解要标什么)
2. [环境与安装检查](#2-环境与安装检查)
3. [启动两路驱动](#3-启动两路驱动)
4. [检查数据](#4-检查数据)
5. [采集五组](#5-采集五组)
6. [预处理和旧数据隔离](#6-预处理和旧数据隔离)
7. [人工初值](#7-人工初值)
8. [精优化](#8-精优化)
9. [如何验收](#9-如何验收)
10. [导入 FAST-LIVO2](#10-导入-fast-livo2)
11. [重新采集与故障速查](#11-重新采集与故障速查)

## 1. 先理解要标什么

| 项目 | 本轮做法 | 是否已经独立验证 |
|---|---|---|
| RGB 相机内参 K、畸变 D | 使用对应相机、对应分辨率的出厂 CameraInfo | 只核对数值，未独立检验精度 |
| 相机和雷达空间外参 | Koide 工具，多场景静止数据 + 人工初值 + 精优化 | 已生成候选值，需投影/独立场景验证 |
| 雷达和内置 IMU 外参 | 当前配置保留参考值 | Mid-360S 本机安装与参考定义仍需核实 |
| RGB–雷达时间偏移 | 当前 0 秒 | 未标定，静止场景无法检验运动时的错时 |
| D435i 左右红外/深度/相机 IMU | 本轮不接入 | 此 FAST-LIVO2 流程无需它们 |

FAST-LIVO2 用单路 RGB 提供纹理，用雷达提供三维几何，用雷达内部 IMU 提供惯性信息，因此图像窗口只有一个视角是正常的。不要把“单目图像”理解成必须依靠双目深度。

**单位和方向最容易出错：**

- Koide `T_lidar_camera`：`p_lidar = R_lc × p_camera + t_lc`，即 RGB 光学系到雷达。
- JSON 数组顺序：`[tx, ty, tz, qx, qy, qz, qw]`；前三个量单位为**米**。
- FAST-LIVO2：`p_camera = Rcl × p_lidar + Pcl`，需要前面变换的逆。
- `Rcl = R_lcᵀ`，`Pcl = -R_lcᵀ × t_lc`。只改变平移正负号不能完成求逆。
- RGB 光学坐标：x 向右、y 向下、z 向前。不要把相机外壳坐标或 `camera_link` 当作光学系。

平移的长度 `sqrt(tx²+ty²+tz²)` 是两坐标原点间的距离。0.13 m = 13 cm；1.3 m = 130 cm；它不是雷达到墙面的距离。机械尺量通常也没有准确定位两个坐标原点，所以仅能做合理性检查。

## 2. 环境与安装检查

### 2.1 机械要求

将两传感器固定在同一刚性支架；轻拉线缆不应让相对角度变化。RGB 尽量朝向雷达有效视场中的有纹理区域；没有适用于所有场景的“最佳倾角”。避免镜头被雷达/支架遮挡，也避免雷达近场打在支架上。室内行走通常先让 RGB 前向、略向下能看到墙面和物体，实测共同视场后再决定角度。

安装完成后再标定；拆装、调整倾角或松动后必须重新标定。录静态标定包时可以改变整个支架的位姿，但一包录制过程中必须保持静止。

### 2.2 每个新终端都加载环境

以下路径是**本机真实位置**。复制到新机器时只需把第一行路径换成克隆后的路径；外部雷达驱动工作区默认 `/home/hzw/livox_ws`，如位置变化，先 `export LIVOX_DRIVER_WS=实际路径`。

```bash
# source 只加载 ROS 环境、定义变量，不启动任何节点。
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0
cd "$LIVO_WS"
printf '工作区：%s\n数据目录：%s\n' "$LIVO_WS" "$CALIB_ROOT"

# 已安装软件检查；某项不存在，先按调试手册的环境章节处理。
ros2 pkg prefix fast_livo
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix realsense2_camera
command -v docker
command -v xauth
python3 -c 'import yaml, numpy, scipy, rosbag2_py; print("Python 依赖可用")'
docker info --format '{{.ServerVersion}}'
free -h
df -h "$LIVO_WS"
```

本机脚本默认 `CALIB_ROOT=$LIVO_WS/calibration_data/rigid_mount_01`。五包 RGB 原始数据占数 GB，转换和预处理还会多占磁盘；建议至少预留 15 GB。`source` 提示文件不存在时先修正路径，不要继续把空变量传给命令。

### 2.3 网络与进程检查

```bash
# 本机雷达网卡应有 192.168.1.50/24，雷达为 192.168.1.114。
ip -br addr show eno1
ping -c 3 -W 1 192.168.1.114

# 列出当前 ROS 节点；尚未启动驱动时没有输出是正常的。
ros2 node list
```

先在原终端 Ctrl+C 关闭 Viewer2、FAST-LIO2、FAST-LIVO2 和旧雷达驱动。不要同时启动两个雷达驱动。ping 成功只证明网络可达，不代表正在发布 ROS 数据。

## 3. 启动两路驱动

后文的 A、B、C 是三个独立终端：A 保持雷达运行，B 保持相机运行，C 做检查和录包。不要在仍被驱动占用的终端继续粘贴下一条命令。

### 3.1 终端 A：标定用 PointCloud2 雷达驱动

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0
# 标定工具需要 PointCloud2；本启动文件设置 xfer_format=0。
ros2 launch fast_livo calibration_mid360s_points.launch.py
```

如果报告缺少 `MID360s_config.json`，先定位实际驱动包：

```bash
# 在检查终端执行，不在正在运行驱动的终端执行。
ros2 pkg prefix --share livox_ros_driver2
cat "$(ros2 pkg prefix --share livox_ros_driver2)/config/MID360s_config.json"
```

应匹配本机 host IP `.50`、雷达 IP `.114`，并使用该型号的配置。驱动内置外参保留零，避免它先变换一次坐标。不要用其他设备配置覆盖整个 JSON。

### 3.2 终端 B：D435i 只开彩色流

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  serial_no:=_261722072542 \
  enable_color:=true rgb_camera.color_profile:=640x480x30 \
  enable_depth:=false enable_infra:=false enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  enable_rgbd:=false enable_sync:=false align_depth.enable:=false
```

序列号前的 `_` 是驱动要求的数字字符串写法。整个标定、验证、建图保持同一分辨率；更换相机或分辨率后不得继续沿用旧 K。

## 4. 检查数据

终端 C：

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
export ROS_DOMAIN_ID=0
mkdir -p "$CALIB_ROOT/bags"
ros2 node list
ros2 topic info /livox/lidar --verbose
ros2 topic info /camera/camera/color/image_raw --verbose

# 以下 timeout 只停止查询命令；8 秒后返回 124 是正常的。
timeout --signal=INT 8s ros2 topic hz /livox/lidar
timeout --signal=INT 8s ros2 topic hz /livox/imu
timeout --signal=INT 8s ros2 topic hz /camera/camera/color/image_raw

# 保存本次内参，后续可与配置核对。
ros2 topic echo /camera/camera/color/camera_info --once | tee "$CALIB_ROOT/camera_info.yaml"
ros2 topic echo /livox/lidar --once --field fields
```

| 检查项 | 正常现象 | 不满足时 |
|---|---|---|
| `/livox/lidar` 类型 | `sensor_msgs/msg/PointCloud2`，约 10 Hz，1 个发布者 | 关闭 CustomMsg 驱动，重开 A |
| 点云字段 | x、y、z、intensity | 核对驱动与 xfer_format |
| `/livox/imu` | 约 200 Hz | 先排驱动问题 |
| RGB 图像 | `sensor_msgs/msg/Image`，约 30 Hz，1 个发布者 | 核对相机、USB、序列号和分辨率 |
| CameraInfo | 640×480、plumb_bob | 核对配置，不能硬填旧 K |

本次 K：fx=608.162231、fy=607.041321、cx=323.260315、cy=251.738312；CameraInfo 给出的 D 为零。这是出厂模型，并不证明真实镜头完全没有畸变。若边缘持续有系统性弯曲，再做独立 RGB 内参标定，并统一工具与融合端的相机模型。

预览（另一个已加载环境的终端）：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/camera/camera/color/image_raw`。场景应清晰、有门框/箱子/墙角且不过曝。检查后关闭预览，减少录包时的负载。

## 5. 采集五组

### 5.1 如何摆放

本方法无需标定板。已有四圆/AprilTag 板可作为场景纹理，不会自动检测其标记，也不需要输入板尺寸。

| 组别 | 建议场景变化 | 录制时 |
|---|---|---|
| pose_01 | 正视门框、箱子、墙角 | 放稳后静止约 20 秒 |
| pose_02 | 整套支架向左换观察位置 | 同上 |
| pose_03 | 向右换观察位置 | 同上 |
| pose_04 | 改变整体俯仰，看到高低结构 | 同上 |
| pose_05 | 换距离/方向，补充近远物体 | 同上 |

近处约 1–2 m、远处约 3–5 m 的结构都进入共同视场。避免五组只有同一块平板、无纹理白墙、玻璃、镜面或移动的人。不要让相机和雷达相对移动。

### 5.2 逐组录制

终端 C 已加载环境；A、B 保持运行。每次执行一条，录完再移动支架：

```bash
# 组 1：放稳后执行，脚本自动约 20 秒结束，不覆盖同名包。
cd "$LIVO_WS"
bash scripts/record_calibration_pose.sh pose_01
```

移动整套设备，放稳后执行：

```bash
bash scripts/record_calibration_pose.sh pose_02
```

换观察位置、放稳：

```bash
bash scripts/record_calibration_pose.sh pose_03
```

换角度、放稳：

```bash
bash scripts/record_calibration_pose.sh pose_04
```

换距离/方向、放稳：

```bash
bash scripts/record_calibration_pose.sh pose_05
```

不要一次把五条粘贴运行，否则来不及更换场景。脚本用 SIGINT 正常收尾；输出每包信息。每包应有图像、CameraInfo、雷达、IMU 四个话题，点云百余帧、图像数百帧。没有数据或明显丢包时先解决再处理。

```bash
# 完整检查五包，不会回放或修改它们。
for n in 01 02 03 04 05; do
  ros2 bag info "$CALIB_ROOT/bags/pose_$n"
done
du -sh "$CALIB_ROOT/bags"
```

完成后在 A、B 各 Ctrl+C 停止驱动，关闭图像预览、旧标定窗口，再离线处理。

## 6. 预处理和旧数据隔离

```bash
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
cd "$LIVO_WS"
# 使用预编译镜像，不在本机编译标定依赖。
docker pull "$CALIB_IMAGE"
docker image inspect "$CALIB_IMAGE" --format '{{.Id}}' > "$CALIB_ROOT/solver_image_id.txt"

# 1 CPU、5 GiB 容器内存；脚本显式指定本机三个话题。
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 15 \
  python3 scripts/preprocess_calibration.py

ls -lh "$CALIB_ROOT/preprocessed"
python3 -m json.tool "$CALIB_ROOT/preprocessed/calib.json"
```

该脚本的行为：

1. 检查 `pose_01` 至 `pose_05` 的话题、类型和消息数。
2. 如误录为 CustomMsg，流式转换至新的 `bags_points_*`，保留原包不动。只用于**静止标定**，省略逐点时间、tag、line；不能拿转换包验证运动去畸变。
3. 输出到新的 `preprocessed_build_*`，避免工具跳过已有 PNG/PLY 导致继续读旧数据。
4. 验证五组非空 PNG/PLY、JSON 场景列表和输入文件未变动，记录镜像 ID、来源文件元信息。
5. 成功后把旧 `preprocessed` 改名归档，再启用本轮输出；旧融合 YAML 也移至 `old_visual_config_*.yaml`，防止误用。失败保留旧结果。

**成功门槛：**出现 `Verified and activated five NEW datasets`；当前目录有五组图像/点云和 `calib.json`。此时还没有精外参，属于正常状态。不要在重新录包后跳过本步骤，也不要继续使用旧 GUI 窗口。

Docker 报权限错误时先处理用户的 Docker 访问权限；不要随意给授权文件全局写权限。预处理失败查看 `$CALIB_ROOT/preprocessed_build_*.log`，修复后重新执行脚本即可产生新目录。

## 7. 人工初值

在**本机图形桌面的终端**执行；SSH 无 DISPLAY 的终端不能直接打开此 X11 窗口：

```bash
cd "$LIVO_WS"
# 脚本读取桌面 X11 cookie，写入仅当前用户可读的 docker.xauth。
# 不使用 xhost +，不要把 docker.xauth 上传或发给别人。
bash scripts/calib_gui.sh initial_guess_manual
```

如果 DISPLAY 为空或 X11 授权为空，先检查桌面环境：

```bash
printf 'DISPLAY=%s\nXAUTHORITY=%s\n' "${DISPLAY:-}" "${XAUTHORITY:-}"
command -v xauth
```

不要把 `xauth list` 的 cookie 内容贴到网上；本机记录是 X11，此入口不宣称支持所有 Wayland 环境。

### 7.1 窗口内逐步操作

1. 保持图像 `0 deg`，通过 `bag_id` 查看场景。
2. 右键点云中的真实几何角点，再右键图像里的**同一个物理点**。
3. 点 **Add picked points** 加入一对；不是选完就自动加入。
4. 重复约 6–10 对；选点分散到图像上下左右和不同距离。不要只选同一条边或同一小块平面，不要把反光边当物体边。
5. 点 **Estimate**，调 `blend_weight` 检查投影方向和轮廓。
6. 点 **Save**，确认终端/界面提示保存 `/data/calib.json`，再关闭窗口。

五组共用一套外参。可在一组取得可靠初值，也可切换 `bag_id` 补充其他组的点对，再统一 Estimate/Save；每一对的 2D 和 3D 点必须来自同一组。无需对五组分别保存五套外参。左下 `clear` 是清日志；想重新选点，可靠做法是关窗重开。

### 7.2 先检查平移量级

```bash
python3 - <<'PY'
import json, math, os
from pathlib import Path
p=Path(os.environ['CALIB_ROOT'])/'preprocessed/calib.json'
d=json.loads(p.read_text())
for key in ('init_T_lidar_camera', 'T_lidar_camera'):
    if key in d.get('results', {}):
        t=d['results'][key][:3]
        length=math.sqrt(sum(v*v for v in t))
        print(key, '平移(m)=', t, '原点间距(cm)=', length*100)
PY
```

本支架约 13 cm，初值 17 cm 可以继续做几何检查；如果得到 1–3 m，先检查错误点对、深度、内参和图像方向，不要指望精优化修复很差的初值。不要为了凑 13 cm 人工缩放或改平移。

## 8. 精优化

```bash
# 使用 calib.json 中列出的全部五组；不要加 --first_n_bags 1。
bash scripts/calib_gui.sh calibrate --auto_quit

# 精优化后重新开查看器，逐组检查，不沿用旧窗口。
bash scripts/calib_gui.sh viewer
```

`f` 是优化目标值，**不是像素误差、厘米误差或准确率**。本次第一轮目标约 4.7475→4.7353；仅表示该轮目标下降，不能由下降百分比推断精度。不同外层采样/积分阶段的 f 不保证可直接横比。

出现 `Wolfe zoom phase failed...` 表示当前线搜索未找到满足条件的步长；不等于立即崩溃，也不能直接忽略后宣告成功。应检查命令是否返回、JSON 是否新增 `results.T_lidar_camera`、最终几何是否合理。若只保留人工初值或投影明显错位，停止导入；先改善场景/初值/内参再试。

```bash
# T_lidar_camera 必须存在；只存在 init_T_lidar_camera 不算完成精优化。
python3 - <<'PY'
import json, os
from pathlib import Path
p=Path(os.environ['CALIB_ROOT'])/'preprocessed/calib.json'
d=json.loads(p.read_text())
print('结果键：', list(d.get('results', {})))
print('精外参：', d['results']['T_lidar_camera'])
PY
```

## 9. 如何验收

| 层次 | 要检查什么 | 当前 13.42 cm 结果 |
|---|---|---|
| 数值有效 | 有精外参、数值有限、四元数单位长度、旋转合法 | 已核对 |
| 安装合理 | 平移模长与机械安装同量级 | 13.4195 cm，机械估测约 13 cm |
| 全部五组投影 | 门框、箱边、墙角在近远距离和图像边缘都一致 | 仍需逐组确认 |
| 独立场景 | 未参与优化的数据也能投影对齐 | 待验证 |
| 重复性 | 另选初值/另一批场景得到接近外参 | 待验证 |
| 静态融合 | 初始化后持续静置，位姿波动满足任务要求 | 已观察明显波动，未通过 |
| 动态融合 | 慢走、转动后墙面不重影、轨迹连续 | 待验证 |
| 时间一致性 | 运动快慢改变时不出现系统性错位 | RGB 时偏未标定 |

建议另采两个未参与求解的静止场景做留出验证：放到**单独的会话/目录**预处理，不把它们混入原五组训练目录。将原候选 `results.T_lidar_camera` 导入留出数据的 `calib.json` 后只运行 viewer，不重新 calibrate；否则变成重新拟合，不是独立验证。更高要求时应使用人工检查点计算投影误差分布，而不是只看彩色大致重合。

当前存档包含静置结果：约 27 秒内 z 范围约 24.9 cm、最大离起点约 18.6 cm。末端位移只有约 2.4 cm 不能掩盖中间波动。该现象需要继续区分外参、时序、IMU、可观测性等原因，不能仅凭它认定外参全错或算法正确。

## 10. 导入 FAST-LIVO2

```bash
cd "$LIVO_WS"
# 脚本拒绝覆盖已有文件，也拒绝与保存相机模型明显不一致的内参/畸变。
# 每次用新名称，方便对比、回滚。
CONFIG_NEW="$LIVO_WS/src/FAST-LIVO2/config/mid360s_d435i_candidate_$(date +%Y%m%d_%H%M%S).yaml"
python3 scripts/apply_camera_lidar_calibration.py \
  "$CALIB_ROOT/preprocessed/calib.json" "$CONFIG_NEW"
printf '本次融合请使用：%s\n' "$CONFIG_NEW"
```

此脚本会求逆并设置 `common.img_en=1`；不会标定雷达–IMU，也不会求时间偏移。用新配置运行时显式传 `visual_config_file:="$CONFIG_NEW"`，按[调试手册](FASTLIVO2_DEBUG_README.md)切回 CustomMsg 驱动。新终端中 `CONFIG_NEW` 不会自动继承，要重新填写刚输出的完整文件路径。

如果只想恢复已存档的 13.42 cm 候选，见存档 README；无需重新采集/预处理，更不能为了启动它先执行会归档当前 YAML 的预处理脚本。

## 11. 重新采集与故障速查

### 11.1 重采五组，保留旧证据

关闭旧驱动、旧 GUI。给新一轮一个新目录；所有相关终端都使用**相同**的目录变量。

```bash
# 示例：先在每个采集/处理终端执行这一行，再 source 环境。
export CALIB_ROOT=/home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/calibration_data/rigid_mount_02
source /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/calibration_env.sh
mkdir -p "$CALIB_ROOT/bags"
printf '%s\n' "$CALIB_ROOT"
```

再从第 3 步执行到第 10 步。脚本当前处理固定五个名称 `pose_01..05`，不要只录 `pose_06` 然后以为它会自动选择最近五包。环境脚本尊重已导出的 CALIB_ROOT；想恢复默认时先 `unset CALIB_ROOT`，再 source。

### 11.2 快速排查

| 现象 | 优先操作 |
|---|---|
| ping 通、node list 空 | 启动驱动；所有终端统一 source、DOMAIN_ID、RMW；检查原终端是否报错 |
| 同名数据报已存在 | 使用新会话目录，避免覆盖 |
| 处理的还是旧图片 | 关闭旧 GUI；执行新的预处理脚本；检查 `preprocessed_sources.json`；重新开 GUI |
| CustomMsg 不被工具识别 | 用脚本离线转换静止包；不要修改原包 |
| GUI 起不来 | 本机桌面 X11、DISPLAY、xauth；不要 xhost + |
| 初值平移变成几米 | 检查对应点是否真是同物理点、K、组别、图像旋转 |
| 一组对齐其他组不齐 | 检查安装是否松动、录制是否移动、场景退化、畸变模型 |
| 静止看着好、转动重影 | 检查时间偏移、曝光、滚动快门和去畸变输入 |
| 修改 YAML 后没变化 | 启动时明确绝对路径，查 ros2 param get；关闭旧节点再启动 |

## 参考与方法边界

[Koide 官方采集说明](https://koide3.github.io/direct_visual_lidar_calibration/collection/)建议 Livox 静止积累并采多场景；本手册选 20 秒/组留出启动余量。[官方程序说明](https://koide3.github.io/direct_visual_lidar_calibration/programs/)定义点对初值、精优化与 `T_lidar_camera` 方向。本机命令、坐标求逆和验收状态由当前代码/配置核对。

[参考仓库 KaiFeng-Frank/lidar-camera-imu-calibration](https://github.com/KaiFeng-Frank/lidar-camera-imu-calibration)可作为完整标定链的学习材料，但其设备身份、ROS 版本、K/D、外参和时间偏移不能直接移植。本工程图像时偏的消费方式是 `corrected = raw - img_time_offset`，不要照搬另一份实现的加号定义。
