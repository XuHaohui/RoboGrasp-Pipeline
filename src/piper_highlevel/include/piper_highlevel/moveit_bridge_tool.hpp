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
                    bool open,
                    moveit::planning_interface::MoveGroupInterface::Plan& plan_out);

bool cartesianMove(moveit::planning_interface::MoveGroupInterface& move_group,
                   const rclcpp::Logger& logger,
                   const geometry_msgs::msg::Pose& target_pose,
                   moveit_msgs::msg::RobotTrajectory& traj_out);

bool allowGripperCollision(
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const rclcpp::Logger& logger,
    bool allow);

bool closeGripperToObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                          double object_width,
                          moveit::planning_interface::MoveGroupInterface::Plan& plan_out);

bool attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const rclcpp::Logger& logger,
                  bool allow_collision);

void addCylinder(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 const rclcpp::Logger& logger,
                 const geometry_msgs::msg::Pose& bottom_pose,
                 const std::string& frame_id);

bool releaseAtPlaceAndLift(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const rclcpp::Logger& logger,
    const std::string& frame_id,
    double joint_delta_tol,
    double pose_pos_tol);

std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(const geometry_msgs::msg::Pose& base_pose);

std::vector<geometry_msgs::msg::Pose> generatePlaceCandidates(const geometry_msgs::msg::Pose& base_pose);

}  // namespace moveit_bridge_tool
