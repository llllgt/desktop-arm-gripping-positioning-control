# ROS 2 运行与验证

本项目使用 ROS 2 Jazzy。任务节点发送轨迹和夹爪 action，控制节点运行 MuJoCo 并发布实际关节状态及工具、工件位姿。Windows 下已验证双进程搬运和安装后的 launch；Ubuntu 移植步骤及 RViz 配置尚未实测。

## 接口

| 类型 | 名称 |
|---|---|
| FollowJointTrajectory action | `/arm_controller/follow_joint_trajectory` |
| GripperCommand action | `/gripper_controller/gripper_cmd` |
| JointState topic | `/joint_states` |
| PoseStamped topic | `/desktop_arm/tcp_pose`、`/desktop_arm/object_pose` |
| String topic | `/desktop_arm/phase` |
| Trigger service | `/desktop_arm/save_results`、`/desktop_arm/shutdown` |

控制桥接受从零时刻开始的四关节位置采样轨迹，以线性插值驱动模型。当前实现覆盖本任务使用的 action 子集，完整容差语义、实机驱动和 ros2_control 插件未实现。

## Windows 环境

先按 [Jazzy Windows 安装说明](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html) 配置运行时、Pixi 和兼容的 Python。还需安装或构建 control_msgs。目录由使用者选择，脚本不固定盘符；以下名称表示所需的目录结构：

```text
<ROS_DEPENDENCY_WORKSPACE>/
  pixi.toml
  pixi.lock
  ros2-windows/local_setup.bat
  control_ws/install/local_setup.bat
```

在仓库根目录调用：

```powershell
.\scripts\run_ros.ps1 -RosHome '<ROS_DEPENDENCY_WORKSPACE>' -Output results/ros2-run
```

`-RosHome` 填写上述依赖工作区的实际路径，也可通过 `DESKTOP_ARM_ROS_HOME` 环境变量提供。Pixi 默认从 PATH 查找；不在 PATH 时加 `-Pixi '<PIXI_EXECUTABLE>'`。数值依赖默认使用仓库 `.venv/Lib/site-packages`；外部虚拟环境可通过 `-PythonPackages '<ENV_SITE_PACKAGES>'` 指定。

脚本启动控制与任务两个独立进程，先检查非法目标拒绝和任务取消，再执行搬运。输出包括 summary、trajectory 和 action 检查日志。验证进程使用 `ROS_DOMAIN_ID=47`，运行日志写入指定输出目录。

### control_msgs 构建

在已配置 MSVC、Windows SDK 和 Ninja 的开发者命令提示符中，准备 control_msgs 源码后执行：

```bat
scripts\build_control_msgs.cmd "<ROS_DEPENDENCY_WORKSPACE>" "<CONTROL_MSGS_WORKSPACE>"
```

脚本使用当前命令窗口的编译器与 SDK 环境，构建工作区中的 control_msgs，不负责下载依赖。

### 包安装与 launch

在激活 Pixi、ROS 和 control_msgs 的环境中，将本仓库作为 `ament_python` 包放入 ROS 工作区并运行 `colcon build --merge-install`。激活该工作区的 install 后，在仓库根目录执行：

```bat
set "PYTHONPATH=%CD%\.venv\Lib\site-packages;%PYTHONPATH%"
ros2 launch desktop_arm transfer.launch.py output:=results/ros-launch-run rviz:=false
```

路径含空格或中文时，须核对批处理参数和原生工具的路径编码支持。`rviz:=true` 可启用已有 RViz 配置，但显示效果尚未验证。

## 验证环境记录

2026-10-07 的 Windows 验证使用：

- 官方 Jazzy Windows 二进制，20260128 版本。
- control_msgs 5.10.0，jazzy 分支提交 `2eaaa5440d6dc2291601ea445b83015a663f5f88`。
- VS 2022 Build Tools、Windows SDK 10.0.22621.0、Ninja。
- colcon-core 0.21.3；更新该版本以处理官方依赖中的输出插件兼容问题。

Pixi 配置和锁文件见 `ros-pixi.toml`、`ros-pixi.lock`，Python 快照见 `ros-python-lock.txt`；数值计算依赖见仓库根目录 `requirements-lock.txt`。

双进程结果位于 `results/ros2`：最终位置偏差 0.5650 mm、最大抬升 39.9496 mm，仿真时间 13.550 s，包含启动和 action 检查的墙钟时间约 69.8 s。Windows 定时回调慢于真实时间。

安装后 launch 结果位于 `results/ros_launch`：位置偏差 0.5590 mm、抬升 39.9562 mm，包含 robot_state_publisher 和完整搬运。任务客户端正常结束；清理其他节点时，Windows 将 SIGINT 升级为 SIGTERM，服务节点日志记录退出码 1。公开 launch 日志中的用户目录、电脑名和临时文件路径已替换为占位符，时间戳、消息及退出状态保留。

## Ubuntu 24.04（待验证）

先安装官方 ROS 2 Jazzy，再安装消息包：

```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-trajectory-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-srvs \
  ros-jazzy-robot-state-publisher python3-venv
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .rosvenv
source .rosvenv/bin/activate
python -m pip install -e .
python scripts/verify_ros.py --output results/ros2-run
```

这些步骤根据包依赖整理，尚未在 Ubuntu 上完成 ROS 通信验证。
