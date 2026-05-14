⚠️提示：本仓库仍在开发中，接口与启动方式可能会变化。
如果需要知道当前进展与本版本更新内容，请参考 版本记录 / 路线图 部分

⚠️使用警告：本项目目前稳定度不足，可能出现规划失败的情况。如果在stage2-3失败，请退出再重新开始，这种情况发生概率不高。如果在stage7-8失败，请退出后等待一段时间重新开始。后续会针对这个痛点修改。

# Piper Highlevel + MuJoCo（ROS 2 Humble）

本工程以 **MuJoCo 仿真** 为主要运行形态（默认不涉及实机）。整体是一个基于 ROS 2 Humble 的 colcon 工作区，核心自研包为 `piper_highlevel`。

你还需要一套 Piper 相关的描述/MoveIt/仿真包（上游项目 `piper_ros`）。

- **piper_highlevel**（本仓库自研，位于 `src/piper_highlevel`）：MoveIt Bridge 节点（订阅目标位姿 → 调用 MoveIt 规划 → 发布 `/joint_states`）。
- **piper_ros**：包含 `piper_description`、`piper_with_gripper_moveit`、`piper_no_gripper_moveit`、`piper_mujoco` 等。
  - 如果你 `git clone` 下来 **没有** `src/piper_ros/piper_ros`，请按下文“安装/获取 piper_ros”安装。

## 支持内容（概览）

- MoveIt2 规划与 RViz Demo（有/无夹爪，来自 `piper_ros`）
- `piper_highlevel`：`moveit_bridge` 节点 + 一键 launch（同时启动 `move_group` 与 bridge）
- MuJoCo 可视化仿真：订阅 `/joint_states` 驱动 MuJoCo 模型（来自 `piper_ros/piper_mujoco`）

## 环境要求

- Ubuntu 22.04（或与 Humble 匹配的 Linux）
- ROS 2 Humble
- MoveIt2（Humble）
- 构建工具：`colcon`、`rosdep`

MuJoCo 仿真相关的额外依赖（仅在需要时安装）：

- Python 包：`mujoco_py`（`piper_mujoco` 当前使用该库）
  - 说明：`mujoco_py` 对系统 OpenGL/GLFW 等依赖较敏感；如安装失败，请先按其官方说明补齐系统依赖。

> 本 README 默认不覆盖“真机 CAN 控制”。如果你未来需要真机，请直接参考上游 `piper_ros` 的文档。

## 安装/获取 piper_ros（当你的仓库里没有时）

如果你的工作区 `src/` 下没有 `piper_description`、`piper_with_gripper_moveit`、`piper_mujoco` 等包，建议按下面方式把上游 `piper_ros` 拉到工作区里（示例以 Humble 分支为准）：

```bash
mkdir -p src/piper_ros
git clone https://github.com/agilexrobotics/piper_ros.git src/piper_ros
cd src/piper_ros/piper_ros
git checkout humble
cd ../../../
```

然后你就会拥有 MuJoCo/MoveIt 所需的相关包与资源文件。

## 环境配置

为了方便地在每次使用工作区时设置环境变量，本仓库提供了一个快捷脚本：

```bash
source scripts/setup_env.sh
```

该脚本会自动：
- 设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（使用 CycloneDDS 中间件）
- Source 工作区的 `install/setup.bash`

后续所有涉及 ROS 2 命令的操作都可以用这个一行脚本代替繁琐的手工操作。

## 快速开始（构建工作区）

在工作区根目录执行：

```bash
# 本工程基于 ROS 2 Humble
source /opt/ros/humble/setup.bash

# 1) 安装 ROS 依赖（会从 package.xml 解析）
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2) 安装 Python 依赖（rosdep 不会处理 pip 依赖）
# - 如果你安装了 piper_ros：按其 requirements 安装
if [ -f src/piper_ros/piper_ros/requirements.txt ]; then
  pip3 install -r src/piper_ros/requirements.txt
fi

# - MuJoCo（piper_mujoco 目前使用 mujoco_py）
pip3 install mujoco_py

# 3) 构建
colcon build --symlink-install

# 4) 生效工作区环境
source scripts/setup_env.sh
```

只构建关键包（开发 `piper_highlevel` 时更快）：

```bash
colcon build --symlink-install \
  --packages-select piper_highlevel piper_description piper_with_gripper_moveit
```

## 运行方式

### 1）MoveIt RViz Demo（推荐先验证环境）

有夹爪：

```bash
source scripts/setup_env.sh
ros2 launch piper_with_gripper_moveit demo.launch.py
```

### 2）启动 MoveIt Bridge（同时启动 move_group + bridge）

该方式会从 `piper_description` 与 `piper_with_gripper_moveit` 读取 URDF/SRDF/kinematics 等配置，并启动：

- `moveit_ros_move_group/move_group`
- `piper_highlevel/moveit_bridge`（节点名：`piper_moveit_bridge`）

启动：

```bash
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

发布一个目标位姿（示例）：

```bash
source scripts/setup_env.sh
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{\
  header: { frame_id: 'world' },\
  pose: {\
    position: { x: 0.35, y: 0.05, z: 0.15 },\
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }\
  }\
}" -1
```

说明：

- `piper_moveit_bridge.launch.py` 默认会发布一个 `world -> base_link` 的静态 TF；如你在自己的 TF 树里已提供对应变换，可按需移除。
- 若你的 planning group 不是 `arm`，可这样指定：

```bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py group_name:=<你的group>
```

更详细的 bridge 文档：

- `src/piper_highlevel/README.md`
- `src/piper_highlevel/README_MOVEIT_BRIDGE.md`

### 3）MuJoCo 仿真（无实机）

MuJoCo 节点会订阅 `/joint_states` 并驱动 MuJoCo 模型渲染；你可以用 MoveIt Bridge 规划后发布的 `/joint_states` 来驱动仿真。

终端 A：启动 MoveIt（move_group + bridge）：

```bash
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

终端 B：启动 MuJoCo 可视化（有夹爪）：

```bash
source scripts/setup_env.sh
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

终端 C：发布目标位姿触发规划（同上节示例）。

注意：

- `piper_mujoco` 依赖 `piper_description` 包中 `mujoco_model/piper_description.xml`；若提示找不到模型文件，请确认 `piper_description` 已正确构建并被 `source scripts/setup_env.sh`。
- MuJoCo Viewer 需要图形界面（X11/Wayland + OpenGL）。在无桌面环境下运行需要额外配置虚拟显示。

## 目录结构

- `src/piper_highlevel/`：高层控制与 MoveIt Bridge（本仓库自研包）
- `src/piper_ros/piper_ros/`：Piper 描述/MoveIt/MuJoCo 等（上游依赖；若你的 clone 没有，需要按上文安装）
- `install/`、`build/`、`log/`：colcon 生成目录

## 常见问题（FAQ）

1）`demo.launch.py` 报“参数需要 double，却给了 string”

通常与本地语言区域（小数点）有关，按 `piper_moveit` 文档建议设置：

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

2）MoveIt 规划失败 / IK 失败

- 确认 `move_group` 已启动且加载了正确的 URDF/SRDF。
- 确认 `frame_id` 与 MoveIt 使用的 planning frame 一致。
- 若 kinematics 超时过低，可在 MoveIt 配置的 `kinematics.yaml` 里增大 `kinematics_solver_timeout`。

3）构建后找不到 launch / urdf 等资源

- 确认执行了 `source install/setup.bash`。
- 若你在 `piper_ros` 内修改了资源文件，推荐使用 `colcon build --symlink-install` 以便开发迭代。

## 版本记录 / 路线图

## 上一版本（已完成）
这次更新添加了一个完整的抓取和放置流水线，让机器人能自动完成从抓取物体到放到指定位置的全过程。主要新增内容包括：

状态机流水线：实现了完整的pick-and-place流程，包含以下状态：

OPEN_GRIPPER：打开夹爪
PRE_GRASP：移动到抓取预备位置
APPROACH：接近物体
GRASP：闭合夹爪抓取
LIFT：抬升物体
PRE_PLACE：移动到放置预备位置
PLACE：放下物体
RELEASE：释放夹爪
RETURN_HOME：返回初始位置
RECOVER：失败恢复机制
MoveIt工具函数库（新增moveit_bridge_tool.cpp）：

夹爪控制：开合夹爪、根据物体尺寸调整夹爪宽度
路径规划：笛卡尔运动、直线移动
碰撞管理：允许/禁止夹爪与物体碰撞
物体管理：添加圆柱体到场景、附加/分离物体到夹爪
候选点生成：自动生成多个抓取和放置角度候选点
过滤器：选择最佳抓取/放置角度，提高成功率
稳定性检测：等待运动稳定、关节变化检测
这些功能让机器人能够更智能地处理复杂的抓取任务，自动尝试不同角度，提高整体成功率。

改用：CycloneDDS
添加放置后后撤

## 下一版本（计划）
添加回到home位置之后关闭加爪的功能，实现流程可循环

## 许可证

本工作区可能会组合多个 ROS 包/上游依赖，许可证以各包 `package.xml` 与上游仓库自带 LICENSE 为准；例如 `src/piper_ros/piper_ros/LICENSE`。
