#include "piper_highlevel/moveit_bridge_fsm.hpp"

#include <cmath>
#include <chrono>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "piper_highlevel/moveit_bridge_tool.hpp"

namespace piper_highlevel {

namespace {

constexpr double kPosePosTol = 0.02;
constexpr double kPoseAngleTol = 0.25;
constexpr double kJointDeltaTol = 0.002;
constexpr int kMaxRetries = 3;

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

MoveItBridgeFsm::MoveItBridgeFsm(
    moveit::planning_interface::MoveGroupInterface& move_group,
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client,
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client,
    const std::string& group_name,
    const rclcpp::Logger& logger)
    : move_group_(move_group),
      gripper_group_(gripper_group),
      planning_scene_interface_(planning_scene_interface),
      get_scene_client_(std::move(get_scene_client)),
      apply_scene_client_(std::move(apply_scene_client)),
      group_name_(group_name),
      logger_(logger)
{

}

bool MoveItBridgeFsm::Run(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id)
{
    (void)frame_id;

    struct PipelineContext {
        geometry_msgs::msg::Pose target_pose;
        std::string frame_id;
        std::vector<double> home_joints;
        std::vector<geometry_msgs::msg::Pose> grasp_candidates;
        std::vector<geometry_msgs::msg::Pose> place_candidates;
        size_t grasp_idx = 0;
        size_t place_idx = 0;
        int retry_count = 0;
        RobotState failed_state = RobotState::IDLE;
        bool collision_allowed = false;
    };

    PipelineContext ctx;
    ctx.target_pose = target_pose;
    ctx.frame_id = frame_id;
    ctx.home_joints = move_group_.getCurrentJointValues();

    geometry_msgs::msg::Pose grasp_seed = target_pose;
    grasp_seed.position.z += (CYLINDER_H + 0.05);
    ctx.grasp_candidates = moveit_bridge_tool::generateGraspCandidates(grasp_seed);

    geometry_msgs::msg::Pose place_seed = target_pose;
    place_seed.position.x = 0.40;
    place_seed.position.y = 0.05;
    place_seed.position.z = CYLINDER_H / 2.0 + 0.12;
    ctx.place_candidates = moveit_bridge_tool::generatePlaceCandidates(place_seed);

    RobotState current_state = RobotState::OPEN_GRIPPER;
    RobotState last_stable_state = RobotState::IDLE;

    auto fail = [&](const char* reason) {
        RCLCPP_WARN(logger_, "State %d failed: %s", static_cast<int>(current_state), reason);
        ctx.failed_state = current_state;
        current_state = RobotState::RECOVER;
    };

    auto plan_and_execute_pose = [&](const geometry_msgs::msg::Pose& target) -> bool {
        move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_res = move_group_.plan(plan);
        if (plan_res != moveit::core::MoveItErrorCode::SUCCESS ||
            plan.trajectory_.joint_trajectory.points.empty()) {
            return false;
        }

        const auto exec_res = move_group_.execute(plan);
        return (exec_res == moveit::core::MoveItErrorCode::SUCCESS);
    };

    while (current_state != RobotState::IDLE && current_state != RobotState::FAILED) {
        switch (current_state) {
        case RobotState::OPEN_GRIPPER: {
            RCLCPP_INFO(logger_, "State: OPEN_GRIPPER");
            auto before = gripper_group_.getCurrentJointValues();
            const bool ok = moveit_bridge_tool::controlGripper(gripper_group_, logger_, true);
            auto after = gripper_group_.getCurrentJointValues();
            if (ok && jointsUpdated(before, after, kJointDeltaTol)) {
                last_stable_state = RobotState::OPEN_GRIPPER;
                current_state = RobotState::PRE_GRASP;
            } else {
                fail("gripper open failed");
            }
            break;
        }
        case RobotState::PRE_GRASP: {
            RCLCPP_INFO(logger_, "State: PRE_GRASP (idx=%zu)", ctx.grasp_idx);
            bool moved = false;
            for (; ctx.grasp_idx < ctx.grasp_candidates.size(); ++ctx.grasp_idx) {
                const auto& candidate = ctx.grasp_candidates[ctx.grasp_idx];
                if (!plan_and_execute_pose(candidate)) {
                    continue;
                }
                auto current = move_group_.getCurrentPose().pose;
                if (isPoseClose(current, candidate, kPosePosTol, kPoseAngleTol)) {
                    moved = true;
                    break;
                }
            }

            if (moved) {
                last_stable_state = RobotState::PRE_GRASP;
                current_state = RobotState::APPROACH;
            } else {
                fail("no valid pre-grasp candidate");
            }
            break;
        }
        case RobotState::APPROACH: {
            RCLCPP_INFO(logger_, "State: APPROACH");
            if (!ctx.collision_allowed) {
                ctx.collision_allowed = moveit_bridge_tool::allowGripperCollision(
                    get_scene_client_, apply_scene_client_, logger_, true);
            }

            geometry_msgs::msg::Pose p = move_group_.getCurrentPose().pose;
            p.position.z = target_pose.position.z + (CYLINDER_H / 2.0);

            move_group_.setStartStateToCurrentState();
            const bool ok = moveit_bridge_tool::cartesianMove(move_group_, logger_, p);
            const auto current = move_group_.getCurrentPose().pose;
            const bool pose_ok = std::fabs(current.position.z - p.position.z) <= kPosePosTol;
            if (ok && pose_ok) {
                current_state = RobotState::GRASP;
            } else {
                fail("approach cartesian move failed");
            }
            break;
        }
        case RobotState::GRASP: {
            RCLCPP_INFO(logger_, "State: GRASP");
            auto before = gripper_group_.getCurrentJointValues();
            const bool close_ok = moveit_bridge_tool::closeGripperToObject(gripper_group_, CYLINDER_R * 2);
            auto after = gripper_group_.getCurrentJointValues();
            const bool updated = jointsUpdated(before, after, kJointDeltaTol);
            const bool attach_ok = moveit_bridge_tool::attachObject(planning_scene_interface_, logger_, true);
            const bool attached = waitForAttachedObject(planning_scene_interface_, "target_cylinder", true,
                                                       std::chrono::milliseconds(1500));
            if (close_ok && updated && attach_ok && attached) {
                current_state = RobotState::LIFT;
            } else {
                fail("grasp failed");
            }
            break;
        }
        case RobotState::LIFT: {
            RCLCPP_INFO(logger_, "State: LIFT");
            geometry_msgs::msg::Pose p = move_group_.getCurrentPose().pose;
            p.position.z += 0.03;
            move_group_.setStartStateToCurrentState();
            const bool ok = moveit_bridge_tool::cartesianMove(move_group_, logger_, p);
            const auto current = move_group_.getCurrentPose().pose;
            if (ok && isPoseClose(current, p, kPosePosTol, kPoseAngleTol)) {
                last_stable_state = RobotState::LIFT;
                current_state = RobotState::PRE_PLACE;
            } else {
                fail("lift failed");
            }
            break;
        }
        case RobotState::PRE_PLACE: {
            RCLCPP_INFO(logger_, "State: PRE_PLACE (idx=%zu)", ctx.place_idx);
            bool moved = false;
            for (; ctx.place_idx < ctx.place_candidates.size(); ++ctx.place_idx) {
                const auto& candidate = ctx.place_candidates[ctx.place_idx];
                if (!plan_and_execute_pose(candidate)) {
                    continue;
                }
                auto current = move_group_.getCurrentPose().pose;
                if (isPoseClose(current, candidate, kPosePosTol, kPoseAngleTol)) {
                    moved = true;
                    break;
                }
            }

            if (moved) {
                last_stable_state = RobotState::PRE_PLACE;
                current_state = RobotState::PLACE;
            } else {
                fail("no valid pre-place candidate");
            }
            break;
        }
        case RobotState::PLACE: {
            RCLCPP_INFO(logger_, "State: PLACE");
            geometry_msgs::msg::Pose p = move_group_.getCurrentPose().pose;
            p.position.z = p.position.z - 0.03;
            move_group_.setStartStateToCurrentState();
            const bool ok = moveit_bridge_tool::cartesianMove(move_group_, logger_, p);
            const auto current = move_group_.getCurrentPose().pose;
            if (ok && std::fabs(current.position.z - p.position.z) <= kPosePosTol) {
                current_state = RobotState::RELEASE;
            } else {
                fail("place cartesian move failed");
            }
            break;
        }
        case RobotState::RELEASE: {
            RCLCPP_INFO(logger_, "State: RELEASE");
            auto before = gripper_group_.getCurrentJointValues();
            const bool open_ok = moveit_bridge_tool::controlGripper(gripper_group_, logger_, true);
            auto after = gripper_group_.getCurrentJointValues();
            const bool updated = jointsUpdated(before, after, kJointDeltaTol);
            const bool detach_ok = moveit_bridge_tool::attachObject(planning_scene_interface_, logger_, false);
            const bool detached = waitForAttachedObject(planning_scene_interface_, "target_cylinder", false,
                                                       std::chrono::milliseconds(1500));
            ctx.collision_allowed = false;
            moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, logger_, false);
            if (open_ok && updated && detach_ok && detached) {
                current_state = RobotState::RETURN_HOME;
            } else {
                fail("release failed");
            }
            break;
        }
        case RobotState::RETURN_HOME: {
            RCLCPP_INFO(logger_, "State: RETURN_HOME");
            if (ctx.home_joints.empty()) {
                fail("home joint state empty");
                break;
            }

            move_group_.setStartStateToCurrentState();
            move_group_.setMaxVelocityScalingFactor(0.2);
            move_group_.setMaxAccelerationScalingFactor(0.2);
            move_group_.setJointValueTarget(ctx.home_joints);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            const auto plan_res = move_group_.plan(plan);
            if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
                fail("return home planning failed");
                break;
            }

            const auto exec_res = move_group_.execute(plan);
            if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
                fail("return home execute failed");
                break;
            }

            auto current = move_group_.getCurrentJointValues();
            bool near_home = true;
            if (current.size() == ctx.home_joints.size()) {
                for (size_t i = 0; i < current.size(); ++i) {
                    if (std::fabs(current[i] - ctx.home_joints[i]) > 0.05) {
                        near_home = false;
                        break;
                    }
                }
            } else {
                near_home = false;
            }

            if (!near_home) {
                fail("return home verification failed");
                break;
            }

            moveit_bridge_tool::attachObject(planning_scene_interface_, logger_, false);
            moveit_msgs::msg::CollisionObject remove_obj;
            remove_obj.id = "target_cylinder";
            remove_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            planning_scene_interface_.applyCollisionObject(remove_obj);
            moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, logger_, false);

            current_state = RobotState::IDLE;
            break;
        }
        case RobotState::RECOVER: {
            ctx.retry_count += 1;
            RCLCPP_WARN(logger_, "Recovering from state %d, last stable %d (retry %d/%d)",
                        static_cast<int>(ctx.failed_state), static_cast<int>(last_stable_state),
                        ctx.retry_count, kMaxRetries);

            if (ctx.retry_count > kMaxRetries) {
                current_state = RobotState::FAILED;
                break;
            }

            if (ctx.failed_state == RobotState::PRE_GRASP ||
                ctx.failed_state == RobotState::APPROACH ||
                ctx.failed_state == RobotState::GRASP ||
                ctx.failed_state == RobotState::LIFT) {
                ctx.collision_allowed = false;
                moveit_bridge_tool::allowGripperCollision(get_scene_client_, apply_scene_client_, logger_, false);
                if (ctx.failed_state == RobotState::PRE_GRASP) {
                    ctx.grasp_idx++;
                }
                if (ctx.grasp_idx >= ctx.grasp_candidates.size()) {
                    current_state = RobotState::FAILED;
                } else {
                    current_state = RobotState::PRE_GRASP;
                }
                break;
            }

            if (ctx.failed_state == RobotState::PRE_PLACE ||
                ctx.failed_state == RobotState::PLACE ||
                ctx.failed_state == RobotState::RELEASE) {
                if (ctx.failed_state == RobotState::PRE_PLACE) {
                    ctx.place_idx++;
                }
                if (ctx.place_idx >= ctx.place_candidates.size()) {
                    current_state = RobotState::FAILED;
                } else {
                    current_state = RobotState::PRE_PLACE;
                }
                break;
            }

            if (ctx.failed_state == RobotState::RETURN_HOME) {
                current_state = RobotState::RETURN_HOME;
                break;
            }

            current_state = RobotState::FAILED;
            break;
        }
        case RobotState::FAILED:
        case RobotState::IDLE:
        default:
            current_state = RobotState::FAILED;
            break;
        }
    }

    if (current_state == RobotState::FAILED) {
        RCLCPP_ERROR(logger_, "Pipeline failed after retries");
        return false;
    }

    return true;
}

}  // namespace piper_highlevel
