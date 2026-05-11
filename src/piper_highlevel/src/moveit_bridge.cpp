#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "piper_highlevel/moveit_bridge.hpp"
#include "piper_highlevel/moveit_bridge_tool.hpp"
#include "piper_highlevel/moveit_bridge_fsm.hpp"

using std::placeholders::_1;

namespace {

bool waitForCollisionObject(moveit::planning_interface::PlanningSceneInterface& psi,
                            const std::string& object_id,
                            bool expected_present,
                            std::chrono::milliseconds timeout)
{
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        auto objs = psi.getObjects({object_id});
        const bool present = (objs.find(object_id) != objs.end());
        if (present == expected_present) {
            return true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

}  // namespace
MoveItBridge::MoveItBridge()
: Node("piper_moveit_bridge")
{
    this->declare_parameter<std::string>("group_name", "arm");
    group_name_ = this->get_parameter("group_name").as_string();

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_;

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, std::bind(&MoveItBridge::pose_cb, this, _1), sub_options);
    
    get_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

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

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    moveit_bridge_tool::attachObject(planning_scene_interface_, this->get_logger(), false);
        
    moveit_msgs::msg::CollisionObject remove_obj;
    remove_obj.id = "target_cylinder";
    remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyCollisionObject(remove_obj);
    moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, this->get_logger(), false);

    waitForCollisionObject(planning_scene_interface_, "target_cylinder", false, std::chrono::milliseconds(1500));

    busy_ = true;

    moveit_bridge_tool::addCylinder(planning_scene_interface_, this->get_logger(), msg->pose, msg->header.frame_id);
    GraspSequence(msg->pose, msg->header.frame_id);

    busy_ = false;
}

void MoveItBridge::GraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id)
{
    move_group_->setPoseReferenceFrame(frame_id);
    move_group_->setMaxVelocityScalingFactor(0.05);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    piper_highlevel::MoveItBridgeFsm fsm(*move_group_,
                                         *gripper_group_,
                                         planning_scene_interface_,
                                         get_scene_client_,
                                         apply_scene_client_,
                                         group_name_,
                                         this->get_logger());
    const bool ok = fsm.Run(target_pose, frame_id);
    if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "FSM pipeline failed");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItBridge>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
