⚠️提示：本仓库仍然在开发中，功能不完整且稳定性不足

# Piper High-Level Control

## 项目简介
Piper High-Level Control 是一个基于 ROS 2 Humble 的机器人控制框架，专注于高层次任务规划与执行。该项目集成了 MoveIt! 框架，用于实现复杂的运动规划任务，例如抓取与放置操作。项目的目标是提供一个模块化、可扩展且高效的机器人控制解决方案。

## 功能特性
- **任务规划与执行**：支持抓取与放置任务的完整流水线，包括目标生成、运动规划、碰撞检测与执行。
- **MoveIt! 集成**：利用 MoveIt! 提供的运动规划能力，支持多种机器人模型与末端执行器。
- **多线程执行器**：通过 MultiThreadedExecutor 解决同步服务调用的潜在死锁问题。
- **日志与调试**：详细的日志记录与调试信息，便于开发与问题排查。

## 系统要求
- **操作系统**：Linux
- **ROS 2 版本**：Humble
- **依赖**：
  - MoveIt!
  - rclcpp
  - geometry_msgs

## 安装与构建
1. 克隆仓库：
   ```bash
   git clone <repository_url> ~/piper_control
   cd ~/piper_control
   ```
2. 安装依赖：
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. 构建工作区：
   ```bash
   colcon build
   ```
4. 设置环境：
   ```bash
   source install/setup.bash
   ```

## 使用说明
### 启动 MoveIt! Demo
运行以下命令启动 MoveIt! 演示：
```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

### 发布目标位姿
通过以下命令发布目标位姿：
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{\
  header: { frame_id: 'base_link' },\
  pose: {\
    position: { x: 0.35, y: 0.0, z: 0.15 },\
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }\
  }\
}" -1
```

### 启动高层次控制节点
运行以下命令启动高层次控制节点：
```bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

## 文件结构
- **src/**：源代码目录
  - `piper_highlevel/src/moveit_bridge.cpp`：主要的任务规划与执行逻辑。
  - `piper_highlevel/src/pick_task.cpp`：抓取任务的实现。
- **install/**：构建后的安装目录。
- **log/**：日志文件。

## 已知问题与解决方案
1. **死锁问题**：
   - 问题：同步服务调用可能导致死锁。
   - 解决：引入 MultiThreadedExecutor 并为服务调用分配独立的 CallbackGroup。

2. **IK 失败**：
   - 问题：低超时时间导致 IK 求解失败。
   - 解决：将 `kinematics_solver_timeout` 增加到 0.05 秒。

3. **日志过多**：
   - 问题：某些 ROS 2 日志过于冗长。
   - 解决：使用 `grep -v` 或配置环境变量抑制特定日志。

## 贡献指南
欢迎贡献代码与改进！请遵循以下步骤：
1. Fork 本仓库。
2. 创建新分支：
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. 提交更改并推送到远程仓库。
4. 创建 Pull Request。

## 许可证
本项目遵循 [MIT 许可证](LICENSE)。

## 联系方式
如有任何问题，请联系项目维护者：
- 邮箱：hajimi@example.com
- GitHub Issues：<repository_url>/issues