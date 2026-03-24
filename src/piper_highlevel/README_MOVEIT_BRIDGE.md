# MoveIt Bridge (piper_moveit_bridge)

本文件说明如何构建并运行 `moveit_bridge.cpp`（ROS2 节点 `piper_moveit_bridge`），该节点会：
- 订阅 `/target_pose` (`geometry_msgs/PoseStamped`)，
- 使用 MoveIt 的 `MoveGroupInterface` 对目标位姿进行运动规划，
- 将规划结果的末端关节位置以 `sensor_msgs/JointState` 发布到 `/joint_states`。

---
## 前提条件
- 已安装并配置 ROS2（与你工作空间对应的发行版，例如 Humble）。
- 已安装并能运行 MoveIt2（`move_group`）。
- 机器人 URDF / SRDF 与 MoveIt 配置已加载（能在 MoveIt 中对 `group_name` 做规划）。
- 工作空间位于仓库根目录（例如本仓库），且包含 `piper_highlevel` 包（`moveit_bridge.cpp` 在 `src/piper_highlevel/src`）。

如果不确定，请先能做到：
```bash
# 启动 move_group（取决于你的 MoveIt 配置）
ros2 launch <your_moveit_pkg> <your_move_group_launch>.launch.py
```

---
## 构建步骤
在工作空间根目录下运行（推荐使用符号链接安装，便于开发）：

```bash
# 从工作区根目录
colcon build --symlink-install --packages-select piper_highlevel
# 构建完成后
source install/setup.bash
```

如果你只想查看 `piper_highlevel` 包中有哪些可运行的可执行文件：

```bash
ros2 pkg executables piper_highlevel
```

通常可执行文件名可能是 `moveit_bridge` 或在 `CMakeLists.txt` 中定义的其它名字，执行上一步能确认具体名称。

---
## 运行节点
假设可执行文件名为 `moveit_bridge`，运行方式如下：

```bash
# 启动节点（可选传参设置 planning group）
ros2 run piper_highlevel moveit_bridge --ros-args -p group_name:=arm
```

- `group_name` 参数默认值为 `arm`，如果你的 MoveIt planning group 名称不同，请替换为正确的组名。
- 该节点会在内部为 MoveIt 创建单独的节点 `piper_move_group_client` 来与 MoveIt 通信。

---
## 测试：发布目标位姿示例
在另一个终端（已 source install/setup.bash）发布一个测试位姿：

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{\
  header: { frame_id: 'base_link' },\
  pose: {\
    position: { x: 0.5, y: 0.2, z: 0.6 },\
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }\
  }\
}" -1
```

- 请确保 `frame_id` 与 MoveIt 使用的参考坐标系一致（如 `base_link` 或 `world`）。

可以通过查看 `/joint_states` 来确认是否收到了规划的末态关节目标：

```bash
ros2 topic echo /joint_states
```

---
## 常见问题与排查
- MoveIt 规划失败（日志 `MoveIt plan failed`）
  - 检查 `move_group` 是否运行且加载了正确的 robot_description / SRDF。
  - 确认目标位姿在可达工作空间内。
  - 检查 `frame_id` 是否匹配 MoveIt 的 planning frame；需要做 TF 变换时，确保 TF tree 完整。

- 规划成功但未发布或 `/joint_states` 数据不完整
  - 程序使用轨迹的最后一个点作为末态。如果 `plan.trajectory_.joint_trajectory.points` 为空，会跳过发布。
  - 确认 MoveIt 返回的 `joint_names` 与 `points.back().positions` 长度一致。

- 性能或阻塞
  - MoveIt 的同步 `plan()` 可能比较慢；若需要非阻塞或并行策略，请改为异步规划或在独立线程中处理。

---
## 进阶建议
- 如果需要发布整个轨迹而非仅末态，可遍历 `plan.trajectory_.joint_trajectory.points` 并基于 `time_from_start` 逐点发布或使用 action/trajectory controller。
- 在真实机器人上运行前，先在仿真（如 Gazebo / MuJoCo）中验证轨迹正确性与安全性。

---
文件位置：src/piper_highlevel/src/moveit_bridge.cpp
如需我把 `moveit_bridge.cpp` 生成一个带行注释的版本，我可以替你创建一个副本并把注释写入。