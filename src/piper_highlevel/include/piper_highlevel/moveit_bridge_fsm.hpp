#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

namespace piper_highlevel {

class MoveItBridgeFsm {
public:
    enum class RobotState : uint8_t {
        IDLE,
        OPEN_GRIPPER,
        PRE_GRASP,
        APPROACH,
        GRASP,
        LIFT,
        PRE_PLACE,
        PLACE,
        RELEASE,
        RETURN_HOME,
        RECOVER,
        FAILED
    };

    MoveItBridgeFsm(moveit::planning_interface::MoveGroupInterface& move_group,
                    moveit::planning_interface::MoveGroupInterface& gripper_group,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
                    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
                    const std::string& group_name,
                    const rclcpp::Logger& logger);

    bool Run(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id);

private:
    moveit::planning_interface::MoveGroupInterface& move_group_;
    moveit::planning_interface::MoveGroupInterface& gripper_group_;
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
    std::string group_name_;
    rclcpp::Logger logger_;
};

}  // namespace piper_highlevel
