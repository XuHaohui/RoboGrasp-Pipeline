#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

#include "piper_highlevel/moveit_bridge.hpp"
#include <functional>

MoveItBridge::MoveItBridge()
: Node("piper_moveit_bridge")
{
    this->declare_parameter<std::string>("group_name", "arm");
    group_name_ = this->get_parameter("group_name").as_string();

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, std::bind(&MoveItBridge::pose_cb, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveItBridge::init_move_group, this));

    RCLCPP_INFO(this->get_logger(), "MoveItBridge node created for group '%s'", group_name_.c_str());
}

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

    geometry_msgs::msg::Pose target = msg->pose;
    geometry_msgs::msg::Pose pre_grasp = target;
    pre_grasp.position.z += 0.05; 

    move_group_->setPositionTarget(
        pre_grasp.position.x,
        pre_grasp.position.y,
        pre_grasp.position.z
    );

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (move_group_->plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Pre-grasp plan failed");
        busy_ = false;
        return;
    }

    move_group_->execute(plan1);
    RCLCPP_INFO(this->get_logger(), "Reached pre-grasp");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
        waypoints, 0.01, 0.0, traj);

    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "Cartesian path failed");
        busy_ = false;
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    plan2.trajectory_ = traj;
    move_group_->execute(plan2);

    RCLCPP_INFO(this->get_logger(), "Approach done");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    geometry_msgs::msg::Pose lift_pose = target;
    lift_pose.position.z += 0.10;

    lift_waypoints.push_back(lift_pose);

    moveit_msgs::msg::RobotTrajectory traj2;
    fraction = move_group_->computeCartesianPath(
        lift_waypoints, 0.01, 0.0, traj2);

    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "Lift failed");
        busy_ = false;
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    plan3.trajectory_ = traj2;
    move_group_->execute(plan3);

    RCLCPP_INFO(this->get_logger(), "Lift done");

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