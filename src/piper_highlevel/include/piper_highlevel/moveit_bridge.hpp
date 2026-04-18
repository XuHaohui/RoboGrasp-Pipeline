#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();

private:
    // 回调函数
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // 核心功能拆分
    /**
     * @brief 执行完整的抓取序列：预抓取 -> 接近 -> 抬起
     */
    void executeGraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id);

    /**
     * @brief 移动到指定位姿（标准关节规划/自由规划）
     */
    bool moveToPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief 笛卡尔直线移动
     */
    bool cartesianMove(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief 执行规划好的路径并处理错误
     */
    bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

    std::string group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool busy_ = false;
};
