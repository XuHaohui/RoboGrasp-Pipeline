#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace moveit { namespace planning_interface { class MoveGroupInterface; } }

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();
    

private:
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    std::string group_name_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int busy_ = false;

};
