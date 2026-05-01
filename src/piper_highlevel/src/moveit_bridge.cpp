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
    
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

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
    best_pre_grasp_pose_ = geometry_msgs::msg::Pose(); // 重置最优姿态防止串扰

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

    std::queue<OwnershipTask> task_queue;

    // 1. 封装动作：张开夹爪
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 0: Opening Gripper...");
        move_group_->setStartStateToCurrentState();
        if (controlGripper(true)) return token; 
        return token; 
    });

    // 2. 封装动作：移动到预抓取
    task_queue.push([this, target_pose, frame_id](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 1: Moving to Pre-Grasp...");
        move_group_->setStartStateToCurrentState();
        geometry_msgs::msg::Pose p = target_pose;
        p.position.z += (CYLINDER_H + 0.05);

        const double drop_z = target_pose.position.z + (CYLINDER_H / 2.0);
        planBestGrasp(p, 1, drop_z);
        return token;
    });

    //2.5. 封装动作：允许夹爪与物体发生碰撞（规划时忽略碰撞） 
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 2: Allowing Collision...");
        move_group_->setStartStateToCurrentState();
        allowGripperCollision(true);
        rclcpp::sleep_for(std::chrono::milliseconds(200)); 
        return token;
    });

    // 3. 封装动作：直线下降
    task_queue.push([this, target_pose](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 3: Cartesian Move Down...");
        
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        move_group_->setStartStateToCurrentState();
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
        if (closeGripperToObject(CYLINDER_R * 2)) {
            return token;
        }
        return token;
    });

    // 5. 封装动作：逻辑吸附
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 5: Attaching Object...");
        move_group_->setStartStateToCurrentState();
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
        auto current_p = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose p = current_p;
        p.position.z += 0.03;
        if (cartesianMove(p)) return token;
        return token;
    });

    //7.回到初始位置
    task_queue.push([this,target_pose](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 7: Move To Home Pose...");
        move_group_->setStartStateToCurrentState();

        geometry_msgs::msg::Pose p= target_pose;
        p.position.x = 0.55;
        p.position.y = 0.00;
        p.position.z = CYLINDER_H + 0.03;

        const double drop_z = CYLINDER_H ;
        planBestGrasp(p, 7, drop_z);
        return token;
    });

    //8.cartesian放置
    task_queue.push([this, target_pose](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 8: Cartesian Place...");
        
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        move_group_->setStartStateToCurrentState();
    
        auto current_p = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose p = current_p;
        p.position.z = CYLINDER_H;

        if (cartesianMove(p)) return token;
        return token;
    });

    //9.释放物体
    task_queue.push([this](TaskPtr token) -> TaskPtr {
        RCLCPP_INFO(this->get_logger(), "Stage 9: Releasing Object...");
        move_group_->setStartStateToCurrentState();
        if (attachObject(false)) {
            rclcpp::sleep_for(std::chrono::seconds(2));
            return token;
        }
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
            RCLCPP_ERROR(this->get_logger(), "Pipeline ABORTED at Stage %zu", stage_idx - 1);
            break;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
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

std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(
    const geometry_msgs::msg::Pose& base_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;

     double tcp_offset_z = 0.12;
     double tcp_offset_y = 0.01;

    for (int i = 0; i < 10; ++i) {   
        double yaw = i * M_PI / 10.0;

        tf2::Quaternion q;
        q.setRPY(0, M_PI / 2.0, yaw); 

        tf2::Vector3 offset_local(0, tcp_offset_y, -tcp_offset_z);
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

bool MoveItBridge::planBestGrasp(const geometry_msgs::msg::Pose& pose, uint8_t stage, double drop_z)
{
    move_group_->setStartStateToCurrentState();

    best_traj_ = moveit_msgs::msg::RobotTrajectory();
    auto candidates = generateGraspCandidates(pose);
    double best_fraction = -1.0;
    moveit::planning_interface::MoveGroupInterface::Plan best_plan; 

    if (stage==1)
    {
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
        } else {
            RCLCPP_INFO(this->get_logger(), "选定最优抓取角度，路径跟踪总比例: %.2f%%", best_fraction * 100.0);
        }
    }else{
            for (auto& target : candidates) {
            // --- 模拟 Stage 7: 规划到预放置点 ---
            move_group_->setPoseTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan pre_plan;
            auto plan_res = move_group_->plan(pre_plan);
            if (plan_res != moveit::core::MoveItErrorCode::SUCCESS ||
                pre_plan.trajectory_.joint_trajectory.points.empty()) {
                continue;
            }

            if (pre_plan.trajectory_.joint_trajectory.points.empty()) continue;

            // --- 模拟 Stage 8: 从预放置点直线下降 ---
            moveit::core::RobotStatePtr robot_state = move_group_->getCurrentState();
            robot_state->setJointGroupPositions(group_name_, pre_plan.trajectory_.joint_trajectory.points.back().positions);
            move_group_->setStartState(*robot_state);
            
            std::vector<geometry_msgs::msg::Pose> waypoints_down;
            
            geometry_msgs::msg::Pose drop_pose = target;
            drop_pose.position.z = drop_z;
            waypoints_down.push_back(drop_pose);

            moveit_msgs::msg::RobotTrajectory traj_down;
            double fraction_down = move_group_->computeCartesianPath(
                waypoints_down, 0.01, 0.0, traj_down);
            
            if (fraction_down > best_fraction) {
                best_fraction = fraction_down;
                best_plan = pre_plan;
                best_pre_grasp_pose_ = target;
            }
            
            RCLCPP_INFO(this->get_logger(), "正在尝试放置姿态: X=%.2f, Y=%.2f, Z=%.2f, 下降轨迹比例: %.2f%%", 
                target.position.x, target.position.y, target.position.z, fraction_down * 100.0);
            
            move_group_->setStartStateToCurrentState(); 

            if (fraction_down >= 1.0) break; 
        }

        if (best_fraction <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "所有角度都无法同时完成预放置与直线下降规划！");
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "选定最优放置角度，放置下降跟踪总比例: %.2f%%", best_fraction * 100.0);
        }
    }

    move_group_->setStartStateToCurrentState();
    auto exec_res = move_group_->execute(best_plan);
    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
        best_traj_ = moveit_msgs::msg::RobotTrajectory();
        return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::Pose drop_pose = move_group_->getCurrentPose().pose;
    drop_pose.position.z = drop_z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(drop_pose);

    moveit_msgs::msg::RobotTrajectory traj_down_real;
    double fraction_down_real = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, traj_down_real);
    if (fraction_down_real >= 0.8 && !traj_down_real.joint_trajectory.points.empty()) {
        best_traj_ = traj_down_real;
    } else {
        best_traj_ = moveit_msgs::msg::RobotTrajectory();
        RCLCPP_WARN(this->get_logger(), "直线轨迹现场生成失败/比例过低(%.2f%%)，将回退到实时规划", fraction_down_real * 100.0);
    }

    return true;
}

bool MoveItBridge::cartesianMove(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_->setPlanningTime(5.0);           
    move_group_->setNumPlanningAttempts(5);     
    move_group_->setMaxVelocityScalingFactor(0.05); 
    move_group_->setStartStateToCurrentState();

    if (best_traj_.joint_trajectory.points.size() > 0)
    {
        auto current_joints = move_group_->getCurrentJointValues();
        const auto& traj_start_joints = best_traj_.joint_trajectory.points.front().positions;
        
        bool start_state_match = true;
        if (current_joints.size() == traj_start_joints.size()) {
            double max_diff = 0.0;
            for (size_t i = 0; i < current_joints.size(); ++i) {
                max_diff = std::max(max_diff, std::abs(current_joints[i] - traj_start_joints[i]));
                RCLCPP_INFO(this->get_logger(), "误差为%.4f", max_diff);
            }
        }

        if (start_state_match) {
            RCLCPP_INFO(this->get_logger(), "使用预存的最优笛卡尔轨迹执行...");
            auto result = move_group_->execute(best_traj_);
            best_traj_ = moveit_msgs::msg::RobotTrajectory();
            return result == moveit::core::MoveItErrorCode::SUCCESS;
        }
    }

    best_traj_ = moveit_msgs::msg::RobotTrajectory();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto start_pose = move_group_->getCurrentPose().pose;

    const int num_points = 10;
    for (int i = 1; i <= num_points; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        geometry_msgs::msg::Pose p = start_pose;

        p.position.x =
            start_pose.position.x +
            t * (target_pose.position.x - start_pose.position.x);

        p.position.y =
            start_pose.position.y +
            t * (target_pose.position.y - start_pose.position.y);

        p.position.z =
            start_pose.position.z +
            t * (target_pose.position.z - start_pose.position.z);

        //p.orientation = start_pose.orientation;// 保持原目标姿态（如果你希望末端姿态不变）
        waypoints.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "[cartesianMove] 输入目标位姿: position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

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
        return gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
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

        return planning_scene_interface_.applyAttachedCollisionObject(attached_object);
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
