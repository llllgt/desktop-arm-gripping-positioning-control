# 来源与开发记录

## 上游模型

基础机械臂采用 [ROBOTIS OpenMANIPULATOR-X](https://github.com/ROBOTIS-GIT/open_manipulator)，固定到 jazzy 分支提交 `9f84095404d3e596267cd520e90963011059ebf6`。

| 本仓库文件 | 上游来源 |
|---|---|
| `assets/upstream/robot.xml` | `open_manipulator_description/mujoco/open_manipulator_x/open_manipulator_x.xml` |
| `assets/upstream/robot.urdf` | `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf` |
| `assets/upstream/meshes/*.stl` | OpenMANIPULATOR-X 网格，单位毫米 |

上游文件保留原始副本与 Apache-2.0 许可证，见 `assets/upstream/LICENSE`。

## 项目实现

本仓库增加了夹指背板、接触垫和定位托座的参数化设计，以及运动学、轨迹、接触仿真、ROS 2 控制桥、结构计算和验证脚本。基础机械臂、驱动关节和夹爪连杆沿用上游设计。

代码和文档开发使用了 Codex/AI 辅助。验证范围包括数值模型、接触搬运和 ROS 通信；实物加工、装配及硬件标定尚未开展。

## 接触作业扩展

增加轴向弹簧探头等效模型、恒力表面扫描、位置与力反馈对照、传感器扰动及退让逻辑。接触力由 MuJoCo 物理求解输出，添加噪声、延迟与零偏后形成模拟测量；所有结果均为仿真。该探头尚未完成安装支架与内部导向的详细机械设计。ROS 2 双进程验证覆盖力数据发布、启动和停止退让服务。实验配置和失败案例见 [恒力接触作业](恒力接触作业.md)。

## 连续轨迹扩展

增加七次位姿与关节样条、点间极值检查、速度/加速度/jerk 时间缩放、速度前馈，以及 ROS 五次 Hermite action 插值。空间八字、升高螺旋和多拐弯用于带载运动对照；结果为 MuJoCo 仿真和 Windows ROS 2 通信验证。算法、参数和原始数据见 [连续轨迹控制](连续轨迹控制.md)。
