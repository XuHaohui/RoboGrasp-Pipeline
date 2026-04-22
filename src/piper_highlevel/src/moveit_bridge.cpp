#include <memory>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include "piper_highlevel/moveit_bridge.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using moveit_msgs::msg::MoveItErrorCodes;

MoveItBridge::MoveItBridge()
: Node("piper_moveit_bridge")
{
    this->declare_parameter<std::string>("group_name", "arm");
    group_name_ = this->get_parameter("group_name").as_string();

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", 10, std::bind(&MoveItBridge::pose_cb, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveItBridge::init_move_group, this));

}

void MoveItBridge::init_move_group()
{
    if (move_group_) return;

    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), group_name_);
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "gripper");
        move_group_->setPlanningTime(10.0);
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");
        timer_->cancel();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
    }
}

void MoveItBridge::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!move_group_) {
        RCLCPP_WARN(this->get_logger(), "MoveGroup not yet initialized, ignoring request.");
        return;
    }
    
    if (busy_) {
        RCLCPP_WARN(this->get_logger(), "MoveGroup is busy, ignoring request.");
        return;
    }

    busy_ = true;

    addCylinder(msg->pose, msg->header.frame_id);

    GraspSequence(msg->pose, msg->header.frame_id);

    busy_ = false;
}

void MoveItBridge::addCylinder(const geometry_msgs::msg::Pose& bottom_pose, const std::string& frame_id)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "target_cylinder";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = CYLINDER_H;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = CYLINDER_R;

    geometry_msgs::msg::Pose cylinder_pose = bottom_pose;
    // MoveIt 中圆柱体的中心是在其高度的一半处
    cylinder_pose.position.z += CYLINDER_H / 2.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(this->get_logger(), "Adding cylinder to the scene");
    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void MoveItBridge::GraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id)
{
    move_group_->setPoseReferenceFrame(frame_id);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    std::vector<RobotAction> task_queue;

    // 1. 封装动作：张开夹爪
    task_queue.push_back([this]() { 
        return controlGripper(true); 
    });

    // 2. 封装动作：移动到预抓取
    task_queue.push_back([this, target_pose, frame_id]() {
        geometry_msgs::msg::Pose p = target_pose;
        p.position.z += (CYLINDER_H + 0.05); // 预抓取点在物体上方
        move_group_->setPoseReferenceFrame(frame_id);
        return moveToPose(p);
    });

    // 3. 封装动作：直线下降
    task_queue.push_back([this, target_pose,frame_id]() {
        geometry_msgs::msg::Pose p = target_pose;
        p.position.z += CYLINDER_H +0.2; // 移动到圆柱体中心高度进行抓取
        move_group_->setPoseReferenceFrame(frame_id);
        return cartesianMove(p);
    });

    // 4. 封装动作：闭合夹爪
    task_queue.push_back([this]() {
        return controlGripper(false);
    });

    for (size_t i = 0; i < task_queue.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Executing Stage %zu...", i);

        move_group_->setStartStateToCurrentState();
        task_queue[i]();

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

bool MoveItBridge::controlGripper(bool open)
{
    if (!gripper_group_) return false;

    gripper_group_->setNamedTarget(open ? "open" : "close");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Gripper plan success, executing...");
        return gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Gripper planning failed!");
        return false;
    }
}

bool MoveItBridge::moveToPose(const geometry_msgs::msg::Pose& pose)
{
    move_group_->clearPoseTargets();
    move_group_->setGoalPositionTolerance(0.005);   // 5mm
    move_group_->setGoalOrientationTolerance(0.1); // ~3°
    move_group_->setPositionTarget(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = pose.position.x;
    // target_pose.position.y = pose.position.y;
    // target_pose.position.z = pose.position.z;
    // tf2::Quaternion q;
    // q.setRPY(0, -M_PI/2, 0);
    // target_pose.orientation = tf2::toMsg(q);

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "gripper_base";   // 你的末端
    ocm.header.frame_id = "base_link";

    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);  // 期望方向
    ocm.orientation = tf2::toMsg(q);

    // 👇 给自由度（关键！）
    ocm.absolute_x_axis_tolerance = 0.5;
    ocm.absolute_y_axis_tolerance = 0.5;
    ocm.absolute_z_axis_tolerance = M_PI;

    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);

    move_group_->setPathConstraints(constraints);
    //move_group_->setPoseTarget(target_pose);
    move_group_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        std::string error_msg;
        switch (result.val) {
            case MoveItErrorCodes::PLANNING_FAILED: error_msg = "PLANNING_FAILED"; break;
            case MoveItErrorCodes::INVALID_MOTION_PLAN: error_msg = "INVALID_MOTION_PLAN"; break;
            case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: error_msg = "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"; break;
            case MoveItErrorCodes::CONTROL_FAILED: error_msg = "CONTROL_FAILED"; break;
            case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA: error_msg = "UNABLE_TO_AQUIRE_SENSOR_DATA"; break;
            case MoveItErrorCodes::TIMED_OUT: error_msg = "TIMED_OUT"; break;
            case MoveItErrorCodes::PREEMPTED: error_msg = "PREEMPTED"; break;
            case MoveItErrorCodes::START_STATE_IN_COLLISION: error_msg = "START_STATE_IN_COLLISION"; break;
            case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS: error_msg = "START_STATE_VIOLATES_PATH_CONSTRAINTS"; break;
            case MoveItErrorCodes::START_STATE_INVALID: error_msg = "START_STATE_INVALID"; break;
            case MoveItErrorCodes::GOAL_IN_COLLISION: error_msg = "GOAL_IN_COLLISION"; break;
            case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS: error_msg = "GOAL_VIOLATES_PATH_CONSTRAINTS"; break;
            case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED: error_msg = "GOAL_CONSTRAINTS_VIOLATED"; break;
            case MoveItErrorCodes::GOAL_STATE_INVALID: 
            {
                // 获取当前设置的目标位姿
                auto targets = move_group_->getPoseTargets();
                std::string target_info = "Unknown target";
                if (!targets.empty()) {
                    auto p = targets[0].pose;
                    target_info = "Position: [" + std::to_string(p.position.x) + ", " + 
                                  std::to_string(p.position.y) + ", " + std::to_string(p.position.z) + "]";
                }
                error_msg = "GOAL_STATE_INVALID (IK No Solution or Collision at " + target_info + ")";
                break;
            }
            case MoveItErrorCodes::UNRECOGNIZED_GOAL_TYPE: error_msg = "UNRECOGNIZED_GOAL_TYPE"; break;
            case MoveItErrorCodes::INVALID_GROUP_NAME: error_msg = "INVALID_GROUP_NAME"; break;
            case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS: error_msg = "INVALID_GOAL_CONSTRAINTS"; break;
            case MoveItErrorCodes::INVALID_ROBOT_STATE: error_msg = "INVALID_ROBOT_STATE"; break;
            case MoveItErrorCodes::INVALID_LINK_NAME: error_msg = "INVALID_LINK_NAME"; break;
            case MoveItErrorCodes::NO_IK_SOLUTION: error_msg = "NO_IK_SOLUTION"; break;
            default: error_msg = "UNKNOWN_ERROR"; break;
        }
        RCLCPP_ERROR(this->get_logger(), "Move to pose planning failed! Error: %s (code: %d)", error_msg.c_str(), result.val);
        return false;
    }

    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveItBridge::cartesianMove(const geometry_msgs::msg::Pose& target_pose)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, traj);

    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "Cartesian path only followed %.2f%% of the trajectory", fraction * 100.0);
        return false;
    }

    return move_group_->execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
