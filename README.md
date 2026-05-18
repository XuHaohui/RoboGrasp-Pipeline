# Piper Control — 基于 MoveIt2 的 Piper 机械臂抓取-放置流水线（MuJoCo 仿真）

**[ROS 2 Humble] [MoveIt2] [MuJoCo] [C++]**

> ⚠️ 本仓库仍在开发中，接口与启动方式可能会变化。如有问题可联系：[xuhaohui07@outlook.com](mailto:xuhaohui07@outlook.com)

---

## 1. 项目概述

本项目实现了一条**完整的 Pick-and-Place 流水线**，在 MuJoCo 物理仿真环境中控制 AgileX Piper 机械臂（含夹爪）自主完成从目标检测到物体放置的全过程。

**核心能力：**
- 订阅目标物体位姿 (`/target_pose`)，自动执行抓取-放置全流程
- 基于**有限状态机**的流水线控制，支持失败恢复与重试
- 多候选点生成 + IK 快速预筛选，提升规划成功率与效率
- 笛卡尔空间直线运动、碰撞矩阵动态管理、物体吸附/分离
- 与 MuJoCo 仿真 + RViz 可视化集成

**环境要求:** Ubuntu 22.04, ROS 2 Humble, MoveIt2, `mujoco_py`

---

## 2. 快速开始

### 2.1 克隆仓库并安装依赖

```bash
source /opt/ros/humble/setup.bash

# 克隆本仓库
git clone <your-repo-url> piper_control
cd piper_control

# 获取上游依赖 piper_ros (如尚未存在)
mkdir -p src/piper_ros
git clone https://github.com/agilexrobotics/piper_ros.git src/piper_ros
cd src/piper_ros/piper_ros && git checkout humble && cd ../../../

# 安装系统依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 依赖 (MuJoCo 与 piper_ros 的额外依赖)
pip3 install mujoco_py
[ -f src/piper_ros/piper_ros/requirements.txt ] && \
    pip3 install -r src/piper_ros/piper_ros/requirements.txt

# 构建
colcon build --symlink-install

# 加载环境
source scripts/setup_env.sh
```

仅构建核心包（快速迭代）：

```bash
colcon build -
```

### 2.2 环境配置

`scripts/setup_env.sh` 自动完成：
- 设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（CycloneDDS 中间件）
- Source `install/setup.bash`

**推荐：** 加入 `~/.bashrc` 避免小数点解析问题：
```bash
export LC_NUMERIC=en_US.UTF-8
```

---

## 3. 运行指南

### 3.1 验证环境 — MoveIt RViz Demo

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py
```

### 3.2 启动 Pick-and-Place 流水线

**终端 A：** MoveIt Bridge（move_group + FSM 节点）

```bash
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

**终端 B：** MuJoCo 可视化

```bash
source scripts/setup_env.sh
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

**终端 C：** 发布目标物体位姿（触发全流程）

```bash
source scripts/setup_env.sh
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: { frame_id: 'world' },
  pose: {
    position: { x: 0.35, y: 0.05, z: 0.02 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}" -1
```

**终端 D：** 启动rviz观查规划

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py 
```


流水线将自动执行：张开夹爪 → 移动到抓取预备位 → 笛卡尔接近 → 闭合抓取 → 抬升 → 移动到放置预备位 → 笛卡尔放置 → 释放 → 返回 Home。

### 3.3 自定义 Planning Group（后续开放）

```bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py group_name:=<你的group名>
```

### 3.4 RViz 可视化（可选终端 D）

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py
```

## 4. 版本历史

### v0.6 (当前)

- **IK 快速预筛选**：PRE_GRASP 和 PRE_PLACE 阶段遍历候选点时，先用 KDL 数值 IK (0.1s 超时) 做快速可达性检查，不可达候选直接跳过，避免进入 OMPL 规划引擎，大幅缩短不可达候选的失败耗时
- **放置后后撤**：RELEASE 阶段在放置物体后执行后撤动作，防止直接回 Home 导致与桌面碰撞
- **Home 后闭合夹爪**：RETURN_HOME 完成后闭合夹爪，确保流程可重复执行

### v0.5 — 状态机重构

- **状态机流水线**：引入 11 状态 FSM（OPEN_GRIPPER → PRE_GRASP → APPROACH → GRASP → LIFT → PRE_PLACE → PLACE → RELEASE → RETURN_HOME → RECOVER），彻底修改为半离线规划模式
- **MoveIt 工具函数库**：夹爪控制、笛卡尔运动、碰撞管理、物体吸附/分离、多候选点生成（10 抓取候选 + 27 放置候选）
- **失败恢复机制**：最多 3 次自动重试，根据失败阶段智能回退
- 改用 **CycloneDDS** 中间件，提升通信稳定性

### v0.4 — 抓取稳定性

- 大幅提高 PRE_GRASP → APPROACH 阶段成功率
- 删除 APPROACH 的笛卡尔接近，改用普通规划接近
- PLACE 阶段改为现场计算笛卡尔接近
- 引入 `cartesianMove` 权重控制，阻止夹爪完全关闭问题

---

## 5. 许可证

本工作区组合多个 ROS 包和上游依赖，各包许可证以各自 `package.xml` 及上游仓库 LICENSE 文件为准。

---

*联系: [xuhaohui07@outlook.com](mailto:xuhaohui07@outlook.com)*
