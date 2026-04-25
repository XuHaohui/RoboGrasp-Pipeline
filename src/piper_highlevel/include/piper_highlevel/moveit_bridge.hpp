#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

const double CYLINDER_H = 0.10;
const double CYLINDER_R = 0.01;

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();
    using RobotAction = std::function<bool()>;

    /**
     * @brief 采样多个姿态进行运动规划
     */
    bool moveToPoseSampling(const geometry_msgs::msg::Pose& pose);

private:
    // 回调函数
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // 核心功能拆分
    /**
     * @brief 执行完整的抓取序列   
     */
    void GraspSequence(const geometry_msgs::msg::Pose& target_pose, const std::string& frame_id);

    /**
     * @brief 控制夹爪开合
     */
    bool controlGripper(bool open);

    /**
     * @brief 移动到指定位姿（标准关节规划/自由规划）
     */
    bool moveToPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief 笛卡尔直线移动
     */
    bool cartesianMove(const geometry_msgs::msg::Pose& target_pose);

    /*
     * @brief 允许或禁止夹爪与物体发生碰撞
     */
    bool allowGripperCollision(bool allow);
    
    /*
     * @brief 向场景中添加圆柱体
     */
    void addCylinder(const geometry_msgs::msg::Pose& bottom_pose, const std::string& frame_id);

// grasp candidates 采样函数声明
friend std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(const geometry_msgs::msg::Pose& base_pose);

    std::string group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;

    std::vector<double> full_joint_state_ = std::vector<double>(8, 0.0);

    bool busy_ = false;
    // 补充声明：发布关节状态
    void publishJointState(const std::vector<double>& positions, bool is_gripper);
};
