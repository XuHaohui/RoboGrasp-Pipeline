piper_highlevel — MoveIt Bridge 启动说明
=====================================

简介
-	此 launch 文件会同时启动 MoveIt 的 `move_group` 节点和本包的 `moveit_bridge` 节点，用于将高层控制与 MoveIt 桥接。

前提
-	已安装与工程对应的 ROS 2 发行版（例如 Humble）和 MoveIt2。
-	已在工作区根目录完成构建并 source 了安装环境。

构建（在工作区根目录执行）
```bash
colcon build --packages-select piper_highlevel piper_description piper_with_gripper_moveit --symlink-install
```

运行
```bash
source install/setup.bash
ros2 launch piper_highlevel piper_moveit_bridge.launch.py
```

要点说明
-	Launch 文件路径：src/piper_highlevel/launch/piper_moveit_bridge.launch.py
-	该 launch 会读取 piper_description 包中的 URDF，以及 piper_with_gripper_moveit 包中的 SRDF 与 kinematics 配置。
-	启动的节点：
  - move_group（package: moveit_ros_move_group）
  - piper_moveit_bridge（package: piper_highlevel，可执行: moveit_bridge）
-	如需修改机器人配置或 group 名称，请在相应包的 config/ 文件中或在 launch/源码中调整参数。

故障排查
-	若启动失败，请确认相关包已成功构建并且 `source install/setup.bash` 已执行。
-	检查终端输出的错误信息，常见问题包括路径错误、依赖未安装或可执行文件未编译。

参照文件
-	[piper_moveit_bridge.launch.py](launch/piper_moveit_bridge.launch.py)
