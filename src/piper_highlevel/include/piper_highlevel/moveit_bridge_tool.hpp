#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

const double CYLINDER_H = 0.08;
const double CYLINDER_R = 0.01;

namespace moveit_bridge_tool {

bool controlGripper(moveit::planning_interface::MoveGroupInterface& gripper_group,
                    const rclcpp::Logger& logger,
                    bool open);

bool cartesianMove(moveit::planning_interface::MoveGroupInterface& move_group,
                   moveit_msgs::msg::RobotTrajectory& best_traj,
                   const rclcpp::Logger& logger,
                   const geometry_msgs::msg::Pose& target_pose);

bool allowGripperCollision(
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const rclcpp::Logger& logger,
    bool allow);

bool closeGripperToObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                          double object_width);

bool attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const rclcpp::Logger& logger,
                  bool allow_collision);

void addCylinder(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 const rclcpp::Logger& logger,
                 const geometry_msgs::msg::Pose& bottom_pose,
                 const std::string& frame_id);

std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(const geometry_msgs::msg::Pose& base_pose);

std::vector<geometry_msgs::msg::Pose> generatePlaceCandidates(const geometry_msgs::msg::Pose& base_pose);

bool placefilter(moveit::planning_interface::MoveGroupInterface& move_group,
                 const std::string& group_name,
                 const rclcpp::Logger& logger,
                 const geometry_msgs::msg::Pose& base_pose,
                 double drop_distance);

bool attitudefilter(moveit::planning_interface::MoveGroupInterface& move_group,
                    const std::string& group_name,
                    const rclcpp::Logger& logger,
                    moveit_msgs::msg::RobotTrajectory& best_traj,
                    geometry_msgs::msg::Pose& best_pre_grasp_pose,
                    const geometry_msgs::msg::Pose& pose,
                    uint8_t stage,
                    double drop_z);

}  // namespace moveit_bridge_tool
