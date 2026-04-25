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

    std::vector<RobotAction> task_queue;

    // 1. 封装动作：张开夹爪
    move_group_->setStartState(*move_group_->getCurrentState());
    task_queue.push_back([this]() { 
        return controlGripper(true); 
    });

    // 2. 封装动作：移动到预抓取
    move_group_->setStartState(*move_group_->getCurrentState());
    task_queue.push_back([this, target_pose, frame_id]() {
        geometry_msgs::msg::Pose p = target_pose;
        p.position.z += (CYLINDER_H + 0.05); // 预抓取点在物体上方
        move_group_->setPoseReferenceFrame(frame_id);
        return moveToPoseSampling(p);
    });

    //2.5 
    move_group_->setStartState(*move_group_->getCurrentState());
    task_queue.push_back([this]() {
        allowGripperCollision(true);
        return true;
    });

    // 3. 封装动作：直线下降
    move_group_->setStartState(*move_group_->getCurrentState());
    task_queue.push_back([this, target_pose,frame_id]() {

        auto current_pose_stamped = move_group_->getCurrentPose();
        geometry_msgs::msg::Pose current_p = current_pose_stamped.pose;
        
        geometry_msgs::msg::Pose p = current_p; 
        p.position.z = target_pose.position.z + (CYLINDER_H / 2.0); 

        move_group_->setPoseReferenceFrame(frame_id);
        return cartesianMove(p);
        //return moveToPoseSampling(p);
    });

    // 4. 封装动作：闭合夹爪
    move_group_->setStartState(*move_group_->getCurrentState());
    task_queue.push_back([this]() {
        return closeGripperToObject(CYLINDER_R*2);
    });

    for (size_t i = 0; i < task_queue.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Executing Stage %zu...", i);

        move_group_->setStartState(*move_group_->getCurrentState());
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
        if (gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            auto current = gripper_group_->getCurrentJointValues();
            return true;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Gripper planning failed!");
        return false;
    }
}

std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(
    const geometry_msgs::msg::Pose& base_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;

     double tcp_offset = 0.12;

    for (int i = 0; i < 8; ++i) {   
        double yaw = i * M_PI / 8.0;

        tf2::Quaternion q;
        q.setRPY(0, M_PI / 2.0, yaw);   

        tf2::Vector3 offset_local(0, 0,-tcp_offset);

        tf2::Vector3 offset_world = tf2::quatRotate(q, offset_local);

        geometry_msgs::msg::Pose p;

        p.position.x = base_pose.position.x + offset_world.x();
        p.position.y = base_pose.position.y + offset_world.y();
        p.position.z = base_pose.position.z + offset_world.z();

        //geometry_msgs::msg::Pose p = base_pose;
        p.orientation = tf2::toMsg(q);

        candidates.push_back(p);
    }

    return candidates;
}

bool MoveItBridge::moveToPoseSampling(const geometry_msgs::msg::Pose& pose)
{
    auto candidates = generateGraspCandidates(pose);

    for (auto& target : candidates) {
        move_group_->clearPoseTargets();
        move_group_->setStartState(*move_group_->getCurrentState());
        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            std::string error_msg;

            auto targets = move_group_->getPoseTargets();
            std::string target_info = "Unknown target";
            if (!targets.empty()) {
                auto p = targets[0].pose;
                target_info = "Position: [" + std::to_string(p.position.x) + ", " + 
                              std::to_string(p.position.y) + ", " + std::to_string(p.position.z) + "], " +
                              "Orientation: [" + std::to_string(p.orientation.x) + ", " + std::to_string(p.orientation.y) + ", " +
                              std::to_string(p.orientation.z) + ", " + std::to_string(p.orientation.w) + "]";
            }       
        }

        RCLCPP_INFO(this->get_logger(), "Found valid grasp!");
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        auto current = move_group_->getCurrentJointValues();
        return true;
    }
    return false;
    }

    RCLCPP_ERROR(this->get_logger(), "No valid grasp found!");
    return false;
}

bool MoveItBridge::cartesianMove(const geometry_msgs::msg::Pose& target_pose)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    RCLCPP_INFO(this->get_logger(), "[cartesianMove] 输入目标位姿: position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    // 打印当前关节状态
    auto before_joints = move_group_->getCurrentJointValues();
    std::ostringstream oss;
    oss << "[cartesianMove] 当前关节: ";
    for (size_t i = 0; i < before_joints.size(); ++i) oss << before_joints[i] << (i+1==before_joints.size()?"":" ");
    RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());

    double eef_step = 0.01;
    double jump_thresh =5.0;
    RCLCPP_DEBUG(this->get_logger(), "[cartesianMove] 规划参数: eef_step=%.3f, jump_thresh=%.3f, waypoints=%zu", eef_step, jump_thresh, waypoints.size());

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

    if (fraction < 0.9) {
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
    
    double target_width = object_width - 0.005; 
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
