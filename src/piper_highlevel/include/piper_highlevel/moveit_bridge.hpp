#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();

private:
    // 回调函数
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // 核心功能拆分
    /**
     * @brief 执行完整的抓取序列   
     */
    void GraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id);

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::string group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;

    std::vector<double> full_joint_state_ = std::vector<double>(8, 0.0);
    // 存储用于笛卡尔直线下降的最优轨迹（在 cpp 中被使用）
    moveit_msgs::msg::RobotTrajectory best_traj_;
    // 记录选定的预抓取位姿
    geometry_msgs::msg::Pose best_pre_grasp_pose_;
    
    struct TaskToken {};
    using TaskPtr = std::unique_ptr<TaskToken>;
    // 定义任务函数类型：接收一个 Token，返回一个 Token
    using OwnershipTask = std::function<TaskPtr(TaskPtr)>;

    bool busy_ = false;
};
