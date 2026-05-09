#include <memory>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include "piper_highlevel/moveit_bridge.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <queue>   
#include <memory>  
#include <sstream>

#include <moveit/robot_state/robot_state.h>

using std::placeholders::_1;
using moveit_msgs::msg::MoveItErrorCodes;

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

    attachObject(false); 
        
    moveit_msgs::msg::CollisionObject remove_obj;
    remove_obj.id = "target_cylinder";
    remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyCollisionObject(remove_obj);
    allowGripperCollision(false);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

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
        if (controlGripper(true)) return token; 
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
        attitudefilter(p, 1, drop_z);
        return token;
    });

    //2.5. 封装动作：允许夹爪与物体发生碰撞（规划时忽略碰撞） 
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 2: Allowing Collision...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        allowGripperCollision(true);
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

        if (cartesianMove(p)) return token;
        return token;
    });

    // 4. 封装动作：闭合夹爪
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 4: Closing Gripper to Object...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        if (closeGripperToObject(CYLINDER_R * 2)) {
            return token;
        }
        return token;
    });

    // 5. 封装动作：逻辑吸附
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 5: Attaching Object...");
        move_group_->setStartStateToCurrentState();
        gripper_group_->setStartStateToCurrentState();
        if (attachObject(true)) {
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
        if (cartesianMove(p)) return token;
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

        auto candidates = generatePlaceCandidates(base_pose);
        bool executed = false;

        for (const auto& p : candidates) {
            move_group_->setStartStateToCurrentState();
            move_group_->setPoseTarget(p);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            const auto plan_res = move_group_->plan(plan);
            if (plan_res != moveit::core::MoveItErrorCode::SUCCESS ||
                plan.trajectory_.joint_trajectory.points.empty()) {
                continue;
            }

            const auto exec_res = move_group_->execute(plan);
            if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
                executed = true;
                break;
            }
        }

        if (!executed) {
            RCLCPP_WARN(this->get_logger(), "Pre-place planning/execution failed for all candidates");
        }

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

        if (cartesianMove(p)) return token;
        return token;
    });

    //9.封装动作：释放物体
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 9: Opening Gripper...");
        move_group_->setStartStateToCurrentState();
        if (controlGripper(true)) return token; 
        return token; 
    });

    //10.释放物体
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 10: Releasing Object...");
        move_group_->setStartStateToCurrentState();
        if (attachObject(false)) {
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

        attachObject(false);
        moveit_msgs::msg::CollisionObject remove_obj;
        remove_obj.id = "target_cylinder";
        remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface_.applyCollisionObject(remove_obj);
        allowGripperCollision(false);

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

bool MoveItBridge::controlGripper(bool open)
{
    if (!gripper_group_) return false;

    gripper_group_->setNamedTarget(open ? "open" : "close");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Gripper plan success, executing...");
        if (gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            auto current = gripper_group_->getCurrentJointValues();
            return true;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Gripper planning failed!");
        return false;
    }
    return false;
}

std::vector<geometry_msgs::msg::Pose> MoveItBridge::generateGraspCandidates(
    const geometry_msgs::msg::Pose& base_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;

     double tcp_offset_z = 0.12;
     double tcp_offset_y = 0.01;
     double tcp_offset_x = 0.0;

    for (int i = 0; i < 10; ++i) {   
        double yaw = i * M_PI / 10.0;

        tf2::Quaternion q;
        q.setRPY(0, M_PI / 2.0, yaw); 

        tf2::Vector3 offset_local(tcp_offset_x, tcp_offset_y, -tcp_offset_z);
        tf2::Vector3 offset_world = tf2::quatRotate(q, offset_local);

        geometry_msgs::msg::Pose p = base_pose;
        p.position.x = base_pose.position.x + offset_world.x();
        p.position.y = base_pose.position.y + offset_world.y();
        p.position.z = base_pose.position.z + offset_world.z();

        p.orientation = tf2::toMsg(q);

        candidates.push_back(p);
    }

    return candidates;
}

std::vector<geometry_msgs::msg::Pose> MoveItBridge::generatePlaceCandidates(
    const geometry_msgs::msg::Pose& base_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;

    const double place_offset_x = 0.0;
    const double place_offset_y = 0.0;
    const double place_offset_z = 0.0;

    const int samples = 10;

    for (int i = 0; i < samples; ++i) {
        const double yaw = (2.0 * M_PI) * (static_cast<double>(i) / samples);

        tf2::Quaternion q;
        q.setRPY(0.0, M_PI / 2.0, yaw);

        tf2::Vector3 offset_local(place_offset_x, place_offset_y, -place_offset_z);
        tf2::Vector3 offset_world = tf2::quatRotate(q, offset_local);

        geometry_msgs::msg::Pose p = base_pose;
        p.position.x = base_pose.position.x + offset_world.x();
        p.position.y = base_pose.position.y + offset_world.y();
        p.position.z = base_pose.position.z + offset_world.z();
        p.orientation = tf2::toMsg(q);

        candidates.push_back(p);
    }

    return candidates;
}

bool MoveItBridge::attitudefilter(const geometry_msgs::msg::Pose& pose, uint8_t stage, double drop_z)
{
    move_group_->setStartStateToCurrentState();

    best_traj_ = moveit_msgs::msg::RobotTrajectory();
    double best_fraction = -1.0;
    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    moveit_msgs::msg::RobotTrajectory best_down_traj_sim;
    double best_down_fraction_sim = -1.0;

    auto candidates = generateGraspCandidates(pose);

    for (auto& target : candidates) {
        // --- 模拟 Stage 1: 预抓取点 ---
        geometry_msgs::msg::Pose pre_grasp = target;

        move_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan pre_plan;
        auto plan_res = move_group_->plan(pre_plan);
        if (plan_res != moveit::core::MoveItErrorCode::SUCCESS ||
            pre_plan.trajectory_.joint_trajectory.points.empty()) {
            continue;
        }

        // --- 模拟 Stage 3: 从预抓取点直线下降 ---
        moveit::core::RobotStatePtr robot_state = move_group_->getCurrentState();
        robot_state->setJointGroupPositions(group_name_, pre_plan.trajectory_.joint_trajectory.points.back().positions);
        move_group_->setStartState(*robot_state);

        std::vector<geometry_msgs::msg::Pose> waypoints_down;
        geometry_msgs::msg::Pose drop_pose = pre_grasp;
        drop_pose.position.z = drop_z;
        waypoints_down.push_back(drop_pose);

        moveit_msgs::msg::RobotTrajectory traj_down;
        double fraction_down = move_group_->computeCartesianPath(
            waypoints_down, 0.01, 0.0, traj_down);

        // ---  模拟 Stage 6: 从抓取点抬高 ---
        double fraction_up = 0.0;
        moveit_msgs::msg::RobotTrajectory test_traj_up;
        if (fraction_down >= 1.0) {
            robot_state->setJointGroupPositions(group_name_, traj_down.joint_trajectory.points.back().positions);
            move_group_->setStartState(*robot_state);

            std::vector<geometry_msgs::msg::Pose> waypoints_up;
            geometry_msgs::msg::Pose lift_pose = drop_pose;
            lift_pose.position.z += 0.05;
            waypoints_up.push_back(lift_pose);

            fraction_up = move_group_->computeCartesianPath(waypoints_up, 0.01, 0.0, test_traj_up);
        }

        double total_fraction = fraction_down + fraction_up;
        if (total_fraction > best_fraction) {
            best_fraction = total_fraction;
            best_plan = pre_plan;
            best_pre_grasp_pose_ = pre_grasp;
        }

        RCLCPP_INFO(this->get_logger(), "正在尝试规划到位姿: X=%.2f, Y=%.2f, Z=%.2f",
            pre_grasp.position.x, pre_grasp.position.y, pre_grasp.position.z);

        move_group_->setStartStateToCurrentState();

        if (fraction_down >= 0.9 && fraction_up >= 0.9) break;
    }

    if (best_fraction <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "所有角度都无法规划路径！");
        best_traj_ = moveit_msgs::msg::RobotTrajectory();
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "选定最优抓取角度，路径跟踪总比例: %.2f%%", best_fraction * 100.0);

    move_group_->setStartStateToCurrentState();
    move_group_->execute(best_plan);
    best_traj_ = moveit_msgs::msg::RobotTrajectory();

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::Pose drop_pose = move_group_->getCurrentPose().pose;
    drop_pose.position.z = drop_z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(drop_pose);

    moveit_msgs::msg::RobotTrajectory traj_down_real;
    double fraction_down_real = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, traj_down_real);
        RCLCPP_WARN(this->get_logger(),
        "REAL DOWN fraction = %.3f, points=%zu",
        fraction_down_real,
        traj_down_real.joint_trajectory.points.size());

        if (fraction_down_real >= 0.8 &&!traj_down_real.joint_trajectory.points.empty())
    {
        best_traj_ = traj_down_real;
        RCLCPP_INFO(this->get_logger(),"缓存真实笛卡尔下降轨迹成功");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),"真实笛卡尔下降轨迹生成失败");
        best_traj_ = moveit_msgs::msg::RobotTrajectory();
    }

    return true;
}

bool MoveItBridge::cartesianMove(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_->setPlanningTime(5.0);           
    move_group_->setNumPlanningAttempts(5);     
    move_group_->setMaxVelocityScalingFactor(0.2); 
    move_group_->setGoalTolerance(0.05);
    move_group_->setStartStateToCurrentState();

    constexpr double kAllowedStartTolerance = 0.01;

    if (best_traj_.joint_trajectory.points.size() > 0)
    {
        const auto current_names = move_group_->getJointNames();
        const auto current_vals = move_group_->getCurrentJointValues();

        const auto& traj_names = best_traj_.joint_trajectory.joint_names;
        const auto& traj_start = best_traj_.joint_trajectory.points.front().positions;

        bool start_state_match = true;
        double max_diff = 0.0;

        auto findIndex = [&](const std::vector<std::string>& names, const std::string& name) -> int {
            for (size_t i = 0; i < names.size(); ++i) {
                if (names[i] == name) return static_cast<int>(i);
            }
            return -1;
        };

        if (start_state_match) {
            RCLCPP_INFO(this->get_logger(),
                "预存轨迹起点最大关节偏差: %.5f (allowed_start_tolerance=%.3f)",
                max_diff, kAllowedStartTolerance);
            if (max_diff > kAllowedStartTolerance) {
                start_state_match = false;
                RCLCPP_WARN(this->get_logger(),
                    "预存轨迹起点偏差过大 (%.5f > %.3f)，将执行实时规划",
                    max_diff, kAllowedStartTolerance);
            }
        }

        if (start_state_match) {
            RCLCPP_INFO(this->get_logger(), "使用预存的最优笛卡尔轨迹执行...");
            auto result = move_group_->execute(best_traj_);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                best_traj_ = moveit_msgs::msg::RobotTrajectory();
                return true;
            }
        }
        best_traj_ = moveit_msgs::msg::RobotTrajectory();
    }

    best_traj_ = moveit_msgs::msg::RobotTrajectory();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto start_pose = move_group_->getCurrentPose().pose;
    geometry_msgs::msg::Pose p = start_pose;

    p.position.z = target_pose.position.z;
    waypoints.push_back(p);

    auto before_joints = move_group_->getCurrentJointValues();
    std::ostringstream oss;
    oss << "[cartesianMove] 当前关节: ";
    for (size_t i = 0; i < before_joints.size(); ++i) oss << before_joints[i] << (i+1==before_joints.size()?"":" ");
    RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());

    double eef_step = 0.01;
    double jump_thresh = 0.0;

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

    RCLCPP_INFO(this->get_logger(), "[cartesianMove] computeCartesianPath fraction: %.2f%%, 轨迹点数: %zu", fraction * 100.0, traj.joint_trajectory.points.size());

    if (traj.joint_trajectory.points.size() > 0) {
        const auto& last = traj.joint_trajectory.points.back();
        std::ostringstream oss2;
        oss2 << "[cartesianMove] 末端轨迹点: ";
        for (size_t i = 0; i < last.positions.size(); ++i) oss2 << last.positions[i] << (i+1==last.positions.size()?"":" ");
        RCLCPP_DEBUG(this->get_logger(), "%s", oss2.str().c_str());
    }

    if (fraction < 0.8) {
        RCLCPP_WARN(this->get_logger(), "[cartesianMove] 路径跟踪比例过低: %.2f%%，放弃执行。", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "[cartesianMove] 开始执行轨迹...");
    auto exec_result = move_group_->execute(traj);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        auto current = move_group_->getCurrentJointValues();
        std::ostringstream oss3;
        oss3 << "[cartesianMove] 执行后关节: ";
        for (size_t i = 0; i < current.size(); ++i) oss3 << current[i] << (i+1==current.size()?"":" ");
        RCLCPP_INFO(this->get_logger(), "%s", oss3.str().c_str());
        RCLCPP_INFO(this->get_logger(), "[cartesianMove] Trajectory execution succeeded. Current joint state updated.");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "[cartesianMove] Trajectory execution failed. Error code: %d", exec_result.val);
    }
    return false;
}


bool MoveItBridge::allowGripperCollision(bool allow)
{
    if (!get_scene_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "GetPlanningScene service not available");
        return false;
    }

    if (!apply_scene_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "ApplyPlanningScene service not available");
        return false;
    }

    auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    auto get_future = get_scene_client_->async_send_request(get_req);

    if (get_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "GetPlanningScene timeout");
        return false;
    }

    auto scene = get_future.get()->scene;

    auto& acm = scene.allowed_collision_matrix;

    std::vector<std::string> gripper_links = {
        "gripper_base",
        "link7",
        "link8"
    };

    std::string object_id = "target_cylinder";

    auto ensureEntry = [&](const std::string& name) {
        auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
        if (it == acm.entry_names.end()) {
            acm.entry_names.push_back(name);

            moveit_msgs::msg::AllowedCollisionEntry entry;
            entry.enabled.resize(acm.entry_names.size(), false);

            for (auto& row : acm.entry_values) {
                row.enabled.push_back(false);
            }

            acm.entry_values.push_back(entry);
        }
    };

    ensureEntry(object_id);
    for (auto& link : gripper_links) {
        ensureEntry(link);
    }

    auto getIndex = [&](const std::string& name) {
        return std::distance(acm.entry_names.begin(),
            std::find(acm.entry_names.begin(), acm.entry_names.end(), name));
    };

    int obj_idx = getIndex(object_id);

    for (auto& link : gripper_links) {
        int link_idx = getIndex(link);

        acm.entry_values[obj_idx].enabled[link_idx] = allow;
        acm.entry_values[link_idx].enabled[obj_idx] = allow;
    }

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.allowed_collision_matrix = acm;

    auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_req->scene = diff;

    auto apply_future = apply_scene_client_->async_send_request(apply_req);

    if (apply_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "ApplyPlanningScene timeout");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
        allow ? "ACM updated: ALLOW collision" : "ACM updated: DISALLOW collision");

    return true;
}

bool MoveItBridge::closeGripperToObject(double object_width)
{
    if (!gripper_group_) return false;

    const double MAX_WIDTH = 0.07; 
    
    double target_width = object_width - 0.01; 
    if (target_width < 0) target_width = 0;

    double target_joint_value = target_width / MAX_WIDTH * 0.04; 

    std::vector<double> joint_values = gripper_group_->getCurrentJointValues();
    for(auto &val : joint_values) val = target_joint_value; 

    gripper_group_->setJointValueTarget(joint_values);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        const bool ok = (gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        return ok;
    }
    return false;
}

bool MoveItBridge::attachObject(bool allow_collision)
{
    const std::string object_id = "target_cylinder";

    if (allow_collision)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = "gripper_base";
        attached_object.object.id = object_id;
        attached_object.object.header.frame_id = "gripper_base";
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        attached_object.touch_links = {
            "gripper_base",
            "link7",
            "link8"
        };

        RCLCPP_INFO(this->get_logger(), "Attaching object to gripper...");

        const bool attached_ok = planning_scene_interface_.applyAttachedCollisionObject(attached_object);
        if (attached_ok) {
            moveit_msgs::msg::CollisionObject remove_obj;
            remove_obj.id = object_id;
            remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            planning_scene_interface_.applyCollisionObject(remove_obj);
        }
        return attached_ok;
    }
    else
    {
        moveit_msgs::msg::AttachedCollisionObject detach_object;
        detach_object.object.id = object_id;
        detach_object.link_name = "gripper_base";
        detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        RCLCPP_INFO(this->get_logger(), "Detaching object from gripper...");

        return planning_scene_interface_.applyAttachedCollisionObject(detach_object);
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
