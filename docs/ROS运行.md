# ROS 运行与验证

## 本机可直接复现

Windows 官方 ROS 2 Jazzy 二进制运行时已解压到 `C:\pixi_ws\ros2-windows`，通过 Pixi 的用户目录环境提供依赖。未修改全局 Python、未安装系统服务。项目本身的普通 Python 环境在 `.venv`。

在仓库根目录运行：

```powershell
.\scripts\run_ros.ps1
```

脚本寻找已安装的 `pixi`，或当前工作区中下载的 `../tmp/runtime_downloads/pixi/pixi.exe`；也可通过 `-Pixi` 显式传入路径。它激活 ROS 与 control_msgs，启动两个独立进程，先做 action 拒绝/取消验证，再完整搬运，结果写入 `results/ros2`。预期用时约一分钟，定时器在本机慢于真实时间。

控制端点：

| 类型 | 名称 |
|---|---|
| FollowJointTrajectory action | `/arm_controller/follow_joint_trajectory` |
| GripperCommand action | `/gripper_controller/gripper_cmd` |
| JointState topic | `/joint_states` |
| PoseStamped topic | `/desktop_arm/tcp_pose`、`/desktop_arm/object_pose` |
| String topic | `/desktop_arm/phase` |
| Trigger service | `/desktop_arm/save_results`、`/desktop_arm/shutdown` |

双进程验证使用 `ROS_DOMAIN_ID=47`，避免与同机默认 ROS 节点混用。

## 本机依赖记录

- ROS 2 Jazzy Windows 官方二进制：20260128 版本。
- 官方 Pixi 配置/锁文件：`C:\pixi_ws\pixi.toml`、`pixi.lock`。
- 仓库留有其副本 `docs/ros-pixi.toml` 与 `docs/ros-pixi.lock`；该锁文件反映官方环境，本机后续的 colcon-core pip 更新单独记录在 Python 快照中。
- 本机二进制包不含 control_msgs，从 [ros-controls/control_msgs](https://github.com/ros-controls/control_msgs) jazzy 分支构建；固定提交 `2eaaa5440d6dc2291601ea445b83015a663f5f88`，包版本 5.10.0。
- control_msgs 安装：`C:\pixi_ws\control_ws\install`。
- 编译使用已有 VS 2022 Build Tools、可携式 Windows SDK 10.0.22621.0 与 Ninja；SDK 在 `C:\pixi_ws\windows_sdk`。未安装全局 SDK。
- 本机成功使用的编译脚本保留为 `scripts/build_control_msgs.cmd`；需要先准备上述 SDK、编译器和 control_msgs 源码，脚本本身不下载安装这些依赖。
- 官方依赖中旧 colcon-core 与输出插件不兼容，本机将 colcon-core 更新至 0.21.3；其余 Python 依赖快照见 `ros-python-lock.txt`。
- 项目数值计算依赖来自 `.venv/Lib/site-packages`，版本见仓库根目录 `requirements-lock.txt`。

`C:\pixi_ws\desktop_arm_source` 是指向此项目的目录联接，用于避开 Windows 批处理和某些原生工具对中文路径的编码问题。实际代码仍在原项目目录。

## ROS 包与 launch 验证

本机已成功以 `ament_python` 构建项目，安装在 `C:\pixi_ws\project_ws\install`。在已激活 Pixi、ROS 和 control_msgs 的命令窗口中：

```bat
call C:\pixi_ws\project_ws\install\local_setup.bat
set "PYTHONPATH=C:\pixi_ws\desktop_arm_source\.venv\Lib\site-packages;%PYTHONPATH%"
ros2 launch desktop_arm transfer.launch.py output:=C:/pixi_ws/desktop_arm_source/results/ros_launch rviz:=false
```

实际验证包括：安装后 action 节点、robot_state_publisher、资源加载与完整搬运，最终偏差 0.5590 mm、抬升 39.9562 mm。数据在 `results/ros_launch`。

任务完成后 launch 关闭其他节点；Windows 不支持它使用的 SIGINT 关闭方式，会升级为终止信号，日志可能记录服务节点退出码 1。搬运客户端正常退出，保存的物理结果成功。无 RViz 图形界面验证；`rviz:=true` 及对应配置已提供，但显示效果未实测。

## 另一台 Windows 机器

先按 [官方 Jazzy Windows 二进制安装说明](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html) 配置同一 Python 主版本运行时，安装/构建 control_msgs，再调整 `scripts/ros_runtime.cmd` 的两处安装路径。这里的 `C:\pixi_ws` 和下载工具是本机环境，不会随 Git 仓库上传。ROS 配置复杂于普通仿真，建议先确认 README 中无 ROS 的搬运运行成功。

## Ubuntu 24.04 / ROS 2 Jazzy（未实测）

以下是按包依赖提供的移植步骤，不列为已验证平台。先安装官方 ROS 2 Jazzy，再安装消息依赖：

```bash
sudo apt install ros-jazzy-control-msgs ros-jazzy-trajectory-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-srvs \
  ros-jazzy-robot-state-publisher python3-venv
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .rosvenv
source .rosvenv/bin/activate
python -m pip install -e .
python scripts/verify_ros.py
```

正式部署前核对 DDS、图形驱动和包安装路径。默认验证不渲染、不需要 Gazebo 或 MoveIt。ROS 接口支持本项目所需的轨迹位置采样与夹爪位置指令，未实现完整 action 容差、硬件驱动或 ros2_control 控制器。
