#include "piper_highlevel/moveit_bridge_tool.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

namespace {

bool isPoseClose(const geometry_msgs::msg::Pose& current,
                 const geometry_msgs::msg::Pose& target,
                 double pos_tol,
                 double angle_tol)
{
    const double dx = current.position.x - target.position.x;
    const double dy = current.position.y - target.position.y;
    const double dz = current.position.z - target.position.z;
    const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion q_current;
    tf2::Quaternion q_target;
    tf2::fromMsg(current.orientation, q_current);
    tf2::fromMsg(target.orientation, q_target);
    const double angle_err = q_current.angleShortestPath(q_target);

    return (pos_err <= pos_tol) && (angle_err <= angle_tol);
}

bool jointsUpdated(const std::vector<double>& before,
                   const std::vector<double>& after,
                   double min_delta)
{
    if (before.size() != after.size() || before.empty()) {
        return false;
    }

    for (size_t i = 0; i < before.size(); ++i) {
        if (std::fabs(after[i] - before[i]) > min_delta) {
            return true;
        }
    }

    return false;
}

bool waitUntilStable(moveit::planning_interface::MoveGroupInterface& move_group,
                     std::chrono::milliseconds timeout,
                     std::chrono::milliseconds settle)
{
    const auto start = std::chrono::steady_clock::now();
    auto last_pose = move_group.getCurrentPose().pose;
    auto last_joints = move_group.getCurrentJointValues();
    auto stable_since = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < timeout) {
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        auto now_pose = move_group.getCurrentPose().pose;
        auto now_joints = move_group.getCurrentJointValues();

        const bool pose_stable = isPoseClose(now_pose, last_pose, 0.001, 0.02);
        const bool joints_stable = !jointsUpdated(last_joints, now_joints, 0.0005);

        if (pose_stable && joints_stable) {
            if (std::chrono::steady_clock::now() - stable_since >= settle) {
                return true;
            }
        } else {
            stable_since = std::chrono::steady_clock::now();
        }

        last_pose = now_pose;
        last_joints = now_joints;
    }

    return false;
}

bool waitForAttachedObject(moveit::planning_interface::PlanningSceneInterface& psi,
                           const std::string& object_id,
                           bool expected_attached,
                           std::chrono::milliseconds timeout)
{
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        auto attached = psi.getAttachedObjects({object_id});
        const bool is_attached = (attached.find(object_id) != attached.end());
        if (is_attached == expected_attached) {
            return true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

}  // namespace

namespace moveit_bridge_tool {

bool controlGripper(moveit::planning_interface::MoveGroupInterface& gripper_group,
                    const rclcpp::Logger& logger,
                    bool open,
                    moveit::planning_interface::MoveGroupInterface::Plan& plan_out)
{
    gripper_group.setNamedTarget(open ? "open" : "close");

    if (gripper_group.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Gripper plan success");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Gripper planning failed!");
        return false;
    }
}

bool cartesianMove(moveit::planning_interface::MoveGroupInterface& move_group,
                   const rclcpp::Logger& logger,
                   const geometry_msgs::msg::Pose& target_pose,
                   moveit_msgs::msg::RobotTrajectory& traj_out)
{
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setGoalTolerance(0.05);
    move_group.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose p = start_pose;

    p.position.z = target_pose.position.z;
    waypoints.push_back(p);

    auto before_joints = move_group.getCurrentJointValues();
    std::ostringstream oss;
    oss << "[cartesianMove] 当前关节: ";
    for (size_t i = 0; i < before_joints.size(); ++i) {
        oss << before_joints[i] << (i + 1 == before_joints.size() ? "" : " ");
    }
    RCLCPP_DEBUG(logger, "%s", oss.str().c_str());

    double eef_step = 0.01;
    double jump_thresh = 0.0;

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

    RCLCPP_INFO(logger, "[cartesianMove] computeCartesianPath fraction: %.2f%%, 轨迹点数: %zu",
                fraction * 100.0, traj.joint_trajectory.points.size());

    if (!traj.joint_trajectory.points.empty()) {
        const auto& last = traj.joint_trajectory.points.back();
        std::ostringstream oss2;
        oss2 << "[cartesianMove] 末端轨迹点: ";
        for (size_t i = 0; i < last.positions.size(); ++i) {
            oss2 << last.positions[i] << (i + 1 == last.positions.size() ? "" : " ");
        }
        RCLCPP_DEBUG(logger, "%s", oss2.str().c_str());
    }

    if (fraction < 0.8) {
        RCLCPP_WARN(logger, "[cartesianMove] 路径跟踪比例过低: %.2f%%，放弃执行。", fraction * 100.0);
        return false;
    }

    traj_out = traj;
    return true;
}

bool allowGripperCollision(
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const rclcpp::Logger& logger,
    bool allow)
{
    if (!get_scene_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger, "GetPlanningScene service not available");
        return false;
    }

    if (!apply_scene_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger, "ApplyPlanningScene service not available");
        return false;
    }

    auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    auto get_future = get_scene_client->async_send_request(get_req);

    if (get_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_ERROR(logger, "GetPlanningScene timeout");
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

    auto apply_future = apply_scene_client->async_send_request(apply_req);

    if (apply_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_ERROR(logger, "ApplyPlanningScene timeout");
        return false;
    }

    RCLCPP_INFO(logger,
                allow ? "ACM updated: ALLOW collision" : "ACM updated: DISALLOW collision");

    return true;
}

bool closeGripperToObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                          double object_width,
                          moveit::planning_interface::MoveGroupInterface::Plan& plan_out)
{
    const double MAX_WIDTH = 0.07;

    double target_width = object_width - 0.01;
    if (target_width < 0) {
        target_width = 0;
    }

    double target_joint_value = target_width / MAX_WIDTH * 0.04;

    std::vector<double> joint_values = gripper_group.getCurrentJointValues();
    for (auto& val : joint_values) {
        val = target_joint_value;
    }

    gripper_group.setJointValueTarget(joint_values);

    if (gripper_group.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
    }
    return false;
}

bool attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const rclcpp::Logger& logger,
                  bool allow_collision)
{
    const std::string object_id = "target_cylinder";

    if (allow_collision) {
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

        RCLCPP_INFO(logger, "Attaching object to gripper...");

        const bool attached_ok = planning_scene_interface.applyAttachedCollisionObject(attached_object);
        if (attached_ok) {
            moveit_msgs::msg::CollisionObject remove_obj;
            remove_obj.id = object_id;
            remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            planning_scene_interface.applyCollisionObject(remove_obj);
        }
        return attached_ok;
    }else {
        moveit_msgs::msg::AttachedCollisionObject detach_object;

        detach_object.link_name = "gripper_base";
        detach_object.object.id = object_id;
        detach_object.object.operation =moveit_msgs::msg::CollisionObject::REMOVE;

        RCLCPP_INFO(logger, "Detaching object from gripper...");

        const bool detached_ok =planning_scene_interface.applyAttachedCollisionObject(detach_object);

        return detached_ok;
    }
}

void addCylinder(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 const rclcpp::Logger& logger,
                 const geometry_msgs::msg::Pose& bottom_pose,
                 const std::string& frame_id)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "target_cylinder";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = CYLINDER_H;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = CYLINDER_R;

    geometry_msgs::msg::Pose cylinder_pose = bottom_pose;
    cylinder_pose.position.z += CYLINDER_H / 2.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(logger, "Adding cylinder to the scene");
    planning_scene_interface.addCollisionObjects(collision_objects);
}

bool releaseAtPlaceAndLift(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const rclcpp::Logger& logger,
    const std::string& frame_id,
    double joint_delta_tol,
    double pose_pos_tol)
{
    auto before = gripper_group.getCurrentJointValues();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = controlGripper(gripper_group, logger, true, plan);
    if (!planned) {
        return false;
    }

    gripper_group.execute(plan);
    waitUntilStable(move_group, std::chrono::milliseconds(1500), std::chrono::milliseconds(150));
    auto after = gripper_group.getCurrentJointValues();
    const bool updated = jointsUpdated(before, after, joint_delta_tol);
    if (!updated) {
        return false;
    }

    geometry_msgs::msg::Pose place_bottom;
    auto tcp_pose = move_group.getCurrentPose().pose;
    place_bottom.position = tcp_pose.position;
    place_bottom.position.x += 0.12 ;
    place_bottom.position.y -= 0.02 ;
    place_bottom.position.z -= CYLINDER_H / 2.0;
    place_bottom.orientation.x = 0;
    place_bottom.orientation.y = 0;
    place_bottom.orientation.z = 0;
    place_bottom.orientation.w = 1;
    addCylinder(planning_scene_interface, logger, place_bottom, frame_id);

    const bool detach_ok = attachObject(planning_scene_interface, logger, false);
    const bool detached = waitForAttachedObject(planning_scene_interface, "target_cylinder", false,
                                                std::chrono::milliseconds(1500));
    if (!detach_ok || !detached) {
        return false;
    }

    geometry_msgs::msg::Pose p = move_group.getCurrentPose().pose;
    p.position.z += 0.03;
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    const bool lift_planned = cartesianMove(move_group, logger, p, traj);
    if (!lift_planned) {
        return false;
    }

    move_group.execute(traj);
    waitUntilStable(move_group, std::chrono::milliseconds(2000), std::chrono::milliseconds(200));
    const auto current = move_group.getCurrentPose().pose;
    const bool lifted = (std::fabs(current.position.z - p.position.z) <= pose_pos_tol);
    if (!lifted) {
        return false;
    }

    allowGripperCollision(get_scene_client, apply_scene_client, logger, false);

    geometry_msgs::msg::Pose retreat = move_group.getCurrentPose().pose;
    retreat.position.z -= 0.05;  

    move_group.setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory retreat_traj;
    const bool ok = moveit_bridge_tool::cartesianMove(move_group,logger,retreat,retreat_traj);

    if (!ok) {
        RCLCPP_ERROR(logger, "retreat failed");
        return false;
    }

    move_group.execute(retreat_traj);

    waitUntilStable(move_group,std::chrono::milliseconds(2000),std::chrono::milliseconds(200));

    return true;
}

std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(const geometry_msgs::msg::Pose& base_pose)
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

std::vector<geometry_msgs::msg::Pose>
generatePlaceCandidates(const geometry_msgs::msg::Pose& base_pose)
{
    std::vector<geometry_msgs::msg::Pose> candidates;

    const double dx[] = {0.0, 0.02, -0.02};
    const double dy[] = {0.0, 0.02, -0.02};
    const double yaws[] = {0, M_PI / 3.0, -M_PI / 3.0};

    for (double yaw : yaws) {
        tf2::Quaternion q;
        q.setRPY(0, M_PI / 2.0, yaw);

        for (double x : dx) {
            for (double y : dy) {
                geometry_msgs::msg::Pose p = base_pose;

                p.position.x += x;
                p.position.y += y;
                p.position.z += 0;

                p.orientation = tf2::toMsg(q);

                candidates.push_back(p);
            }
        }
    }

    return candidates;
}

}  // namespace moveit_bridge_tool
