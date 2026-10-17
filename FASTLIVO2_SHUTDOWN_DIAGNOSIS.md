# FAST-LIVO2 自动停止与退出段错误排查（2026-10-17）

用户确认没有按 Ctrl+C，因此历史日志的 `user interrupted with ctrl-c (SIGINT)`
只能证明 launch 收到了 SIGINT，不能证明由用户键盘触发。原日志没有保存信号
发送者，无法事后恢复发送进程身份。静态位姿漂移是另一项待评估问题。

## 已复现的退出缺陷

使用原始 CustomMsg bag `bags/pose_01` 离线回放。回放约 20 秒结束后，节点继续
等待 15 秒，没有自行停止。测试脚本随后向 mapper 主动发送一次 SIGINT，
GDB 捕获到了测试脚本的发送 PID，并复现了 SIGSEGV：

```text
__libc_free
std::_Sp_counted_ptr<pcl::PointCloud<pcl::PointXYZINormal>*>::_M_dispose
LIVMapper::~LIVMapper
main
```

证据保存在 `calibration_data/rigid_mount_01/validation_runs/shutdown_probe_01/`，
包括 GDB 回溯、测试源码、旧二进制和回放日志。这次 SIGINT 是明确标记的诊断
操作，不能用于推断历史那次未知信号是谁发送的。

## 独立最小复现

对系统 PCL 的 VoxelGrid 输入相同的 1,000 个点：

| 编译方式 | Eigen 对齐 | EIGEN_MALLOC_ALREADY_ALIGNED | 退出 |
|---|---:|---:|---:|
| 加 `-march=native` | 32 | 0 | SIGSEGV / −11 |
| 不加该选项 | 16 | 1 | 0 |

这确认了本机发行版 PCL 与 native 构建代码之间的 Eigen 分配/释放 ABI 不一致。
修复移除了 FAST-LIVO2 和 vikit_common 的 `-march=native`，保留正常 Release 优化。
这不是用跳过析构、泄漏点云或强制 `_exit` 来隐藏崩溃。

同时发现源码 `omp_set_num_threads(MP_PROC_NUM)` 会把环境中的 1 线程覆盖成 4。
现改用 CMake 参数 `FAST_LIVO_NUM_THREADS`，默认 1，关闭强制多线程分支。

## 修复后的验证

仅重编译 vikit_common、vikit_ros、fast_livo；顺序单任务、单 CPU，编译子进程
地址空间限制 3 GiB。构建耗时约 2 分 46 秒，无编译错误。

用同一 pose_01 和原 ROS launch 文件验证，包含实际视觉地图点生成。数据回放
结束后继续等待约 40 秒，总观察时间超过 60 秒，未自行停止。诊断脚本明确向
launch 发送 SIGINT 后，mapper 正常退出，退出码 0，未出现 SIGSEGV。
证据：`validation_runs/shutdown_probe_02/summary.json` 与
`fusion_diagnostics_ISEiu0I3/console.log`、`signals.log`。

本次信号跟踪记录 Python 测试脚本 PID 12299 → launch PID 12338 → mapper
PID 12339；同样打印了 `user interrupted with ctrl-c (SIGINT)`。
这实证该文案只是收到 SIGINT 的通用描述，不是键盘操作的证明。
上述已知诊断信号不能反推历史那次未知 SIGINT 的来源。
当前验证修复了可复现的退出段错误，尚未定位用户实测中未知中断的发送者，
也未对静态漂移作出修复结论。

## 捕获未知中断来源

两份传感器驱动保持原来的 A、B 终端运行；终端 C 使用：

```bash
bash /home/hzw/项目文件/四旋翼/livox/fast_livo2_deep_robotics/scripts/run_fusion_diagnostics.sh
```

该脚本没有运行时长限制，不开启 RViz。它跟踪整个 ROS launch 进程树，
将 SIGINT/SIGTERM/SIGSEGV 的发送 PID、信号类型，屏幕输出及进程身份快照保存到
`calibration_data/rigid_mount_01/fusion_diagnostics_*/`。
`console.log` 是运行输出，`signals.log` 是信号证据，`processes.log` 用于查发送 PID。
若信号来自终端内核，si_code/si_pid 也应按其含义解读，不能一概认作普通进程 kill。

此脚本记录本机进程身份，但不记录进程命令参数，减少把敏感参数写入日志的可能。
运行结束后保留这些文件，不要重复启动多份 mapper。
