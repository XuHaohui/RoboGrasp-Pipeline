#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "robograsp_interfaces/msg/object_info.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include "piper_highlevel/moveit_bridge_tool.hpp"
#include "piper_highlevel/gripper_force_ctrl.hpp"

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();

private:
    // 回调函数
    void object_info_cb(const robograsp_interfaces::msg::ObjectInfo::SharedPtr msg);

    // 核心功能拆分
    /**
     * @brief 执行完整的抓取序列   
     */
    void GraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id, const ObjectGeometry& geo);

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::string group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<robograsp_interfaces::msg::ObjectInfo>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;

    std::vector<double> full_joint_state_ = std::vector<double>(8, 0.0);

    bool busy_ = false;
    std::string last_object_id_;

    piper_highlevel::GripperForceController gripper_force_ctrl_;
};
