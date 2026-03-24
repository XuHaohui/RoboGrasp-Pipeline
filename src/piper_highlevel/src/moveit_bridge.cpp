#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

#include "moveit_bridge.hpp"
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>

MoveItBridge::MoveItBridge()
: Node("piper_moveit_bridge")
{
    this->declare_parameter<std::string>("group_name", "arm");
    group_name_ = this->get_parameter("group_name").as_string();

    // 订阅 /target_pose
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, std::bind(&MoveItBridge::pose_cb, this, _1));

    // 发布 /joint_states（可视化用）
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveItBridge::init_move_group, this));

    RCLCPP_INFO(this->get_logger(), "MoveItBridge node created for group '%s'", group_name_.c_str());
}

// 延迟初始化 MoveGroupInterface
void MoveItBridge::init_move_group()
{
    if (move_group_) return;

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), group_name_);

    move_group_->setPlanningTime(10.0);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized");

    timer_->cancel();
}

void MoveItBridge::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!move_group_ || busy_) return;
    busy_ = true;

    RCLCPP_INFO(this->get_logger(), "Received target position");

    move_group_->clearPoseTargets();

    move_group_->setPoseReferenceFrame(msg->header.frame_id);

    move_group_->setPositionTarget(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );

    move_group_->setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Plan failed");
        busy_ = false;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Plan success");

    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Execute failed");
        busy_ = false;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Execution done");

    busy_ = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItBridge>();
    node->init_move_group();  // 延迟初始化 MoveGroupInterface
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}