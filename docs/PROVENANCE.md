# 来源与贡献边界

- 基础机械臂：ROBOTIS OpenMANIPULATOR-X。
- 上游：https://github.com/ROBOTIS-GIT/open_manipulator
- 固定版本：jazzy 分支，`9f84095404d3e596267cd520e90963011059ebf6`。
- `assets/upstream/robot.xml`：上游 `open_manipulator_description/mujoco/open_manipulator_x/open_manipulator_x.xml` 原始副本。
- `assets/upstream/robot.urdf`：上游 `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf` 原始副本。
- `assets/upstream/meshes/*.stl`：上游 OpenMANIPULATOR-X 网格，单位毫米。
- 许可证：Apache-2.0；上游 LICENSE 保留于 `assets/upstream/LICENSE`。

本项目新增：运动学与轨迹规划实现、搬运场景与物理仿真执行器、ROS 2 接口、参数化夹指附件及工件定位托座、设计校核、验证与展示材料。开发由 Codex/AI 辅助完成，用户仍需学习并理解设计、算法和实验结果；不声称从零设计整台机械臂。

参数化附件基于公开 STL 外形拟合，尚未通过实物试装。材料、摩擦和负载是明确的仿真假设；仿真结果不等于实际负载能力或工业精度。
