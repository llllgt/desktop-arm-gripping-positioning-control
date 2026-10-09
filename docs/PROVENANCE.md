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
