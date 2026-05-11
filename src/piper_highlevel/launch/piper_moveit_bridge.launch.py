
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _read_text(path):
    with open(path, "r") as f:
        return f.read()


def _read_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _load_robot_configs():
    # URDF 在 piper_description 包里
    pkg_desc = get_package_share_directory("piper_description")
    urdf_file = os.path.join(pkg_desc, "urdf", "piper_description.urdf")

    # SRDF / Kinematics / Controllers 在 moveit 包里
    pkg_moveit = get_package_share_directory("piper_with_gripper_moveit")
    srdf_file = os.path.join(pkg_moveit, "config", "piper.srdf")
    kinematics_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")
    controllers_file = os.path.join(pkg_moveit, "config", "moveit_controllers.yaml")

    robot_description_content = _read_text(urdf_file)
    robot_description_semantic_content = _read_text(srdf_file)
    robot_description_kinematics_content = _read_yaml(kinematics_file)
    moveit_controllers_content = _read_yaml(controllers_file)

    return {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        "robot_description_kinematics": robot_description_kinematics_content,
        "moveit_controllers": moveit_controllers_content,
    }


def _declare_launch_arguments(ld):
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument("group_name", default_value="arm"))
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_controller_manager",
            default_value="moveit_simple_controller_manager/MoveItSimpleControllerManager",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "planning_plugin",
            default_value="ompl_interface/OMPLPlanner",
        )
    )
    ld.add_action(DeclareLaunchArgument("move_group_delay", default_value="2.0"))
    ld.add_action(DeclareLaunchArgument("bridge_delay", default_value="2.5"))


def _add_static_tf(ld):
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )
    ld.add_action(static_tf_node)


def _add_move_group(ld, cfg, moveit_dict):
    move_group_params = [
        moveit_dict,
        cfg["moveit_controllers"],
        {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "default_planning_pipeline": "ompl",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
    )

    delayed_move_group = TimerAction(
        period=LaunchConfiguration("move_group_delay"),
        actions=[move_group_node],
    )
    ld.add_action(delayed_move_group)


def _add_moveit_bridge(ld, cfg):
    moveit_bridge_node = Node(
        package="piper_highlevel",
        executable="moveit_bridge",
        name="piper_moveit_bridge",
        output="screen",
        parameters=[
            {"robot_description": cfg["robot_description"]},
            {"robot_description_semantic": cfg["robot_description_semantic"]},
            {"robot_description_kinematics": cfg["robot_description_kinematics"]},
            cfg["moveit_controllers"],
            {"group_name": LaunchConfiguration("group_name")},
            {
                "moveit_controller_manager": LaunchConfiguration(
                    "moveit_controller_manager"
                ),
                "robot_controllers": cfg["moveit_controllers"],
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    

    delayed_bridge = TimerAction(
        period=LaunchConfiguration("bridge_delay"),
        actions=[moveit_bridge_node],
    )
    ld.add_action(delayed_bridge)


def generate_launch_description():
    ld = LaunchDescription()

    # 生成 MoveIt 配置（包含 planning_pipelines/ompl）
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    moveit_dict = moveit_config.to_dict()

    cfg = _load_robot_configs()

    _declare_launch_arguments(ld)
    _add_static_tf(ld)
    _add_move_group(ld, cfg, moveit_dict)
    _add_moveit_bridge(ld, cfg)

    return ld