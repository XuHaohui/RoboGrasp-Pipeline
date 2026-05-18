# piper_highlevel

Piper 机械臂高层控制包，提供基于 MoveIt2 的 Pick-and-Place 流水线。

## 依赖

- `rclcpp`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`, `tf2`
- `moveit_ros_planning_interface`, `moveit_msgs`

## 构建

```bash
colcon build 
```

## 启动

```bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py [group_name:=arm]
```

## 节点接口

**节点名:** `piper_moveit_bridge`

| 类型   | 名称           | 消息类型                        |
|--------|----------------|--------------------------------|
| 订阅   | `/target_pose` | `geometry_msgs/msg/PoseStamped` |

| 参数        | 默认值   | 说明                   |
|------------|---------|----------------------|
| `group_name` | `"arm"` | MoveIt planning group |

## 源文件

| 文件                    | 说明                                      |
|------------------------|-------------------------------------------|
| `moveit_bridge.cpp`    | ROS 2 节点，订阅 `/target_pose`，编排流水线    |
| `moveit_bridge_fsm.cpp`| 有限状态机（11 状态），含失败恢复与重试           |
| `moveit_bridge_tool.cpp`| 工具函数库：夹爪控制、笛卡尔运动、碰撞管理、IK 检查、候选点生成 |

## Launch 文件

`piper_moveit_bridge.launch.py` — 一键启动 `move_group` + `piper_moveit_bridge` 节点，自动加载 URDF/SRDF/kinematics 配置。
