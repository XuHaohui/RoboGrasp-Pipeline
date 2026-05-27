# Piper Control — 基于 MoveIt2 的 Piper 机械臂抓取-放置流水线（MuJoCo 仿真）

**[ROS 2 Humble] [MoveIt2] [MuJoCo] [C++]**

> ⚠️ 本仓库已经具备完整流程，但仍然处于优化状态，接口与启动方式可能会变化。如有问题可联系：[xuhaohui07@outlook.com](mailto:xuhaohui07@outlook.com)

---

## 1. 项目概述

本项目实现了一条**完整的 Pick-and-Place 流水线**，在 MuJoCo 物理仿真环境中控制 AgileX Piper 机械臂（含夹爪）自主完成从目标检测到物体放置的全过程。

**核心能力：**
- 订阅目标物体信息 (`/target_pose`)，自动执行抓取-放置全流程
- 基于**有限状态机**（12 状态）的流水线控制，支持失败恢复与重试
- 多候选点生成 + IK 快速预筛选，提升规划成功率与效率
- 笛卡尔空间直线运动、碰撞矩阵动态管理、物体吸附/分离
- 与 MuJoCo 仿真 + RViz 可视化集成（**RViz 必须启动**）

** AI 辅助生成模块：**

`src/piper_highlevel/scripts` 目录中的部分代码主要通过 AI 辅助编程工具生成，
后续由作者进行适配、调试与工程集成。

---

## 2. 快速开始

### 2.1 克隆仓库并安装依赖

```bash
source /opt/ros/humble/setup.bash

# 克隆本仓库
git clone 

# 获取上游依赖 piper_ros (如尚未存在)
cd src
git clone https://github.com/agilexrobotics/piper_ros.git piper_ros
cd piper_ros && git checkout humble

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
colcon build --packages-select piper_highlevel
```

### 2.2 环境配置

`scripts/setup_env.sh` 自动完成：
- 设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（CycloneDDS 中间件）
- 指定 `CYCLONEDDS_URI` 使用 `scripts/cyclonedds.xml`
- Source `install/setup.bash`

**推荐：** 加入 `~/.bashrc` 避免小数点解析问题：
```bash
export LC_NUMERIC=en_US.UTF-8
```

---

## 3. 运行指南

### 3.1 启动 Pick-and-Place 流水线

以下四个终端均需启动，**缺一不可**：

**终端 A：** MoveIt Bridge（move_group + FSM 节点）

```bash
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

**终端 B：** MuJoCo 可视化 + 相机桥接

```bash
source scripts/setup_env.sh
ros2 run piper_highlevel mj_camera_bridge.py
```

**终端 C：** RViz 可视化（**必须启动**，流水线依赖其 move_group 后端）

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py
```

**终端 D：** 发布目标物体信息（触发全流程）

```bash
source scripts/setup_env.sh
ros2 topic pub /target_pose robograsp_interfaces/msg/ObjectInfo "{
  header: { frame_id: 'world' },
  object_class: 'cube',
  bbox_size: [0.04, 0.04, 0.04],
  bottom_center: { x: 0.35, y: 0.05, z: 0.2 }
}" -1
```

流水线将自动执行：张开夹爪 → 移动到抓取预备位 → 笛卡尔接近 → 闭合抓取 → 抬升 → 移动到放置预备位 → 笛卡尔放置 → 释放 → 返回 Home。

### 3.2 验证环境 — MoveIt RViz Demo

可单独启动 RViz 验证 MoveIt 配置是否正常：

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py
```

在 RViz 中可通过拖拽交互球测试规划与运动。

### 3.3 自定义 Planning Group

```bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py group_name:=<你的group名>
```

---

## 4. 节点接口

### 4.1 `piper_moveit_bridge` 节点

| 类型 | 名称 | 消息类型 |
|------|------|----------|
| 订阅 | `/target_pose` | `robograsp_interfaces/msg/ObjectInfo` |

**ObjectInfo 消息字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header.frame_id` | `string` | 坐标系 |
| `object_class` | `string` | 物体类别：`"cube"` / `"box"` / `"sphere"` / 其他(默认圆柱体) |
| `bbox_size` | `float64[3]` | 包围盒尺寸 (x, y, z) |
| `bottom_center` | `geometry_msgs/Point` | 物体底面中心位置 |

### 4.2 `mj_camera_bridge` 节点（MuJoCo 相机桥接）

订阅 `/joint_states` 驱动 MuJoCo 仿真，并发布 RGB/深度图像。内部采用模块化设计：`mj_physics.py` 负责仿真加载与步进，`mj_camera.py` 负责离屏渲染与图像发布，`mj_camera_bridge.py` 负责 ROS 接口编排。

| 类型 | 名称 | 消息类型 |
|------|------|----------|
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 发布 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` (RGB8) |
| 发布 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (32FC1) |
| 发布 | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

### 4.3 静态 TF

Launch 文件自动发布 `world → map` 静态变换。

### 4.4 Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `group_name` | `"arm"` | MoveIt planning group |
| `use_sim_time` | `true` | 使用仿真时间 |
| `moveit_controller_manager` | `moveit_simple_controller_manager/MoveItSimpleControllerManager` | 控制器管理器 |
| `planning_plugin` | `ompl_interface/OMPLPlanner` | 规划算法插件 |
| `move_group_delay` | `2.0` | move_group 延迟启动 (秒) |
| `bridge_delay` | `2.5` | bridge 节点延迟启动 (秒) |

### 4.5 MuJoCo 仿真场景

`config/piper_world.xml` 定义仿真环境，与机器人模型自动合并加载：

| 物体 | 位置 (x, y, z) | 说明 |
|------|----------------|------|
| 桌面 | 0.4, 0, 0.13 | 棕色平面 (0.35×0.35 m) |
| target_cube | 0.35, 0.05, 0.17 | 红色立方体 (0.02 m) |
| target_cylinder | 0.35, -0.07, 0.19 | 绿色圆柱体 (r=0.02, h=0.08) |
| target_box | 0.45, 0.05, 0.175 | 蓝色方块 (0.025 m) |

---

## 5. 有限状态机

流水线由 12 状态 FSM 驱动（`moveit_bridge_fsm.cpp`）：

```
IDLE → OPEN_GRIPPER → PRE_GRASP → APPROACH → GRASP → LIFT
  → PRE_PLACE → PLACE → RELEASE → RETURN_HOME → RECOVER → FAILED
```

- **多候选点生成**：PRE_GRASP 阶段生成 10 个抓取候选位姿，PRE_PLACE 阶段生成 27 个放置候选位姿
- **IK 快速预筛选**：遍历候选点时先用 KDL 数值 IK (0.1s 超时) 做可达性检查，避免进入 OMPL 规划引擎
- **失败恢复**：最多 3 次自动重试，根据失败阶段智能回退到 RECOVER 状态

---

## 6. 版本历史

### v0.7 (当前)

- **接口升级**：订阅类型从 `geometry_msgs/PoseStamped` 升级为 `robograsp_interfaces/ObjectInfo`，支持物体类别、包围盒尺寸、底面中心位置
- **RViz 集成**：RViz 明确为流水线必需节点，不再作为可选组件

### v0.8

- **夹爪摩擦力修复**：运行时修改夹爪 geom 的 friction/solimp/solref/condim 参数，夹持力翻倍，解决抓取物体滑落问题
- **相机桥接模块化**：从单文件拆为 `mj_physics.py`（仿真核心）、`mj_camera.py`（相机渲染）、`mj_camera_bridge.py`（薄壳编排）三个模块，职责分离

### v0.6

- **IK 快速预筛选**：PRE_GRASP 和 PRE_PLACE 阶段遍历候选点时，先用 KDL 数值 IK (0.1s 超时) 做快速可达性检查，不可达候选直接跳过，避免进入 OMPL 规划引擎，大幅缩短不可达候选的失败耗时
- **放置后后撤**：RELEASE 阶段在放置物体后执行后撤动作，防止直接回 Home 导致与桌面碰撞
- **Home 后闭合夹爪**：RETURN_HOME 完成后闭合夹爪，确保流程可重复执行

### v0.5 — 状态机重构

- **状态机流水线**：引入 12 状态 FSM（OPEN_GRIPPER → PRE_GRASP → APPROACH → GRASP → LIFT → PRE_PLACE → PLACE → RELEASE → RETURN_HOME → RECOVER → FAILED），彻底修改为半离线规划模式
- **MoveIt 工具函数库**：夹爪控制、笛卡尔运动、碰撞管理、物体吸附/分离、多候选点生成（10 抓取候选 + 27 放置候选）
- **失败恢复机制**：最多 3 次自动重试，根据失败阶段智能回退
- 改用 **CycloneDDS** 中间件，提升通信稳定性

### v0.4 — 抓取稳定性

- 大幅提高 PRE_GRASP → APPROACH 阶段成功率
- 删除 APPROACH 的笛卡尔接近，改用普通规划接近
- PLACE 阶段改为现场计算笛卡尔接近
- 引入 `cartesianMove` 权重控制，阻止夹爪完全关闭问题

---

## 7. 已知不足

- 抓取控制依赖 MoveIt 的 attach/detach 虚拟绑定，未使用 MuJoCo 接触力反馈
- 当前仅支持圆柱体（cylinder）抓取，其他物体类型待后续适配
- 物理参数与相机参数硬编码在代码中，缺少外部配置文件

---

## 8. 下一步规划

- 改由 MuJoCo 接触力反馈驱动抓取，替换 MoveIt attach/detach
- 扩展支持 cube、box 等更多物体类型
- 物理参数与仿真配置 YAML 化，支持运行时调整

---

## 9. 许可证

本工作区组合多个 ROS 包和上游依赖，各包许可证以各自 `package.xml` 及上游仓库 LICENSE 文件为准。

---

*联系: [xuhaohui07@outlook.com](mailto:xuhaohui07@outlook.com)*
