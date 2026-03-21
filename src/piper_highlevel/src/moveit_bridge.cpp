#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

class MoveItBridge : public rclcpp::Node {
public:
  MoveItBridge()
  : Node("piper_moveit_bridge") {
    this->declare_parameter<std::string>("group_name", "arm");
    group_name_ = this->get_parameter("group_name").as_string();

    // separate node for MoveGroupInterface
    move_group_node_ = rclcpp::Node::make_shared("piper_move_group_client");
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(move_group_node_, group_name_);

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", 10, std::bind(&MoveItBridge::pose_cb, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "moveit_bridge started for group '%s'", group_name_.c_str());
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received target pose, planning...");
    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "MoveIt plan failed");
      return;
    }

    if (plan.trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Planned trajectory empty");
      return;
    }

    const auto &joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const auto &positions = plan.trajectory_.joint_trajectory.points.back().positions;

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_names;
    js.position.assign(positions.begin(), positions.end());

    pub_->publish(js);
    RCLCPP_INFO(this->get_logger(), "Published joint target (from MoveIt) with %zu joints", js.name.size());
  }

  std::string group_name_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
