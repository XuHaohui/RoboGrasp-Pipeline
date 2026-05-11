#include <memory>
#include <vector>
#include <functional>
#include <queue>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "piper_highlevel/moveit_bridge.hpp"
#include "piper_highlevel/moveit_bridge_tool.hpp"

using std::placeholders::_1;
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
    best_traj_ = moveit_msgs::msg::RobotTrajectory();
    best_pre_grasp_pose_ = geometry_msgs::msg::Pose(); 

    moveit_bridge_tool::attachObject(planning_scene_interface_, this->get_logger(), false);
        
    moveit_msgs::msg::CollisionObject remove_obj;
    remove_obj.id = "target_cylinder";
    remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyCollisionObject(remove_obj);
    moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, this->get_logger(), false);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

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

    struct ArmStateSnapshot {
        std::vector<double> joint_values;
    };
    auto home = std::make_shared<ArmStateSnapshot>();

    RCLCPP_INFO(this->get_logger(), "Captured initial arm joint state (%zu joints)", home->joint_values.size());

    std::queue<OwnershipTask> task_queue;

    // 1. 封装动作：张开夹爪
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 0: Opening Gripper...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        if (moveit_bridge_tool::controlGripper(*gripper_group_, this->get_logger(), true)) return token;
        return token; 
    });

    // 2. 封装动作：移动到预抓取
    task_queue.push([this, target_pose, frame_id](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 1: Moving to Pre-Grasp...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        geometry_msgs::msg::Pose p = target_pose;
        p.position.z += (CYLINDER_H + 0.05);

        const double drop_z = target_pose.position.z + (CYLINDER_H / 2.0);
        moveit_bridge_tool::attitudefilter(*move_group_, group_name_, this->get_logger(), best_traj_, best_pre_grasp_pose_, p, 1, drop_z);
        return token;
    });

    //2.5. 封装动作：允许夹爪与物体发生碰撞（规划时忽略碰撞） 
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 2: Allowing Collision...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, this->get_logger(), true);
        rclcpp::sleep_for(std::chrono::milliseconds(200)); 
        return token;
    });

    // 3. 封装动作：直线下降
    task_queue.push([this, target_pose](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 3: Cartesian Move Down...");
        
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        auto current_p = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose p = current_p; 
        p.position.z = target_pose.position.z + (CYLINDER_H / 2.0); 

        if (moveit_bridge_tool::cartesianMove(*move_group_, best_traj_, this->get_logger(), p)) return token;
        return token;
    });

    // 4. 封装动作：闭合夹爪
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 4: Closing Gripper to Object...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        if (moveit_bridge_tool::closeGripperToObject(*gripper_group_, CYLINDER_R * 2)) {
            return token;
        }
        return token;
    });

    // 5. 封装动作：逻辑吸附
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 5: Attaching Object...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        if (moveit_bridge_tool::attachObject(planning_scene_interface_, this->get_logger(), true)) {
            rclcpp::sleep_for(std::chrono::seconds(2));
            return token;
        }
        return token;
    });

    //6. 封装动作：抬高物体
    task_queue.push([this, target_pose](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 6: Lifting...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        auto current_p = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose p = current_p;
        p.position.z += 0.03;
        if (moveit_bridge_tool::cartesianMove(*move_group_, best_traj_, this->get_logger(), p)) return token;
        return token;
    });

    //7.移动到预放置位置（候选姿态，选择第一个可行的）
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 7: Move To Pre-Place Pose...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();

        geometry_msgs::msg::Pose base_pose = move_group_->getCurrentPose().pose;
        base_pose.position.x = 0.40;
        base_pose.position.y = 0.05;
        base_pose.position.z = CYLINDER_H / 2.0 + 0.12;

        moveit_bridge_tool::placefilter(*move_group_, group_name_, this->get_logger(), base_pose, base_pose.position.z - 0.03);

        return token;
    });

    //8.cartesian放置
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 8: Cartesian Place...");
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
    
        auto current_p = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose p = current_p;
        p.position.z = current_p.position.z - 0.03;

        if (moveit_bridge_tool::cartesianMove(*move_group_, best_traj_, this->get_logger(), p)) return token;
        return token;
    });

    //9.封装动作：释放物体
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 9: Opening Gripper...");
        move_group_->setStartStateToCurrentState();
        if (moveit_bridge_tool::controlGripper(*gripper_group_, this->get_logger(), true)) return token;
        return token; 
    });

    //10.释放物体
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 10: Releasing Object...");
        move_group_->setStartStateToCurrentState();
        if (moveit_bridge_tool::attachObject(planning_scene_interface_, this->get_logger(), false)) {
            rclcpp::sleep_for(std::chrono::seconds(2));
            return token;
        }
        return token;
    });

    // 11. 回到初始状态 
    task_queue.push([this, &home](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 11: Return to Initial State & Cleanup...");

        if (home && !home->joint_values.empty()) {
            move_group_->setStartStateToCurrentState();
            move_group_->setMaxVelocityScalingFactor(0.2);
            move_group_->setMaxAccelerationScalingFactor(0.2);

            move_group_->setJointValueTarget(home->joint_values);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                const auto plan_res = move_group_->plan(plan);
                if (plan_res == moveit::core::MoveItErrorCode::SUCCESS) {
                    const auto exec_res = move_group_->execute(plan);
                    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(this->get_logger(), "Return-to-initial execute failed (code=%d)", exec_res.val);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Return-to-initial planning failed");
                }
        }

        moveit_bridge_tool::attachObject(planning_scene_interface_, this->get_logger(), false);
        moveit_msgs::msg::CollisionObject remove_obj;
        remove_obj.id = "target_cylinder";
        remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface_.applyCollisionObject(remove_obj);
        moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, this->get_logger(), false);

        home.reset();
        return token;
    });

    auto master_token = std::make_unique<TaskToken>();

    size_t stage_idx = 0;
    while (!task_queue.empty() && master_token != nullptr) {
        move_group_->setStartStateToCurrentState();
        
        auto current_task = std::move(task_queue.front());
        task_queue.pop();

        master_token = current_task(std::move(master_token));

        if (master_token == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline ABORTED at Stage %zu", stage_idx);
            break;
        }

        ++stage_idx;
        
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    home.reset();
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
