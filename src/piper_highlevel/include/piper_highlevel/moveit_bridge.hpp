#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

const double CYLINDER_H = 0.08;
const double CYLINDER_R = 0.01;

// 定义 home 区域的物理尺寸
const double HOME_X = 0.1;
const double HOME_Y = 0.1;
const double HOME_Z = 0.0; 

class MoveItBridge : public rclcpp::Node {
public:
    MoveItBridge();
    void init_move_group();
    using RobotAction = std::function<bool()>;

    /**
     * @brief 采样多个姿态进行运动规划
     */
    bool attitudefilter(const geometry_msgs::msg::Pose& pose, uint8_t stage, double drop_z);

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

    /**
     * @brief 将夹爪闭合到指定的宽度
     */
    bool closeGripperToObject(double object_width);

    /**
     * @brief 逻辑上将物体附着到夹爪（在规划场景中添加/移除附着物）
     */
    bool attachObject(bool allow_collision);
    
    /*
     * @brief 向场景中添加圆柱体
     */
    void addCylinder(const geometry_msgs::msg::Pose& bottom_pose, const std::string& frame_id);

    bool moveToPoseSampling(const geometry_msgs::msg::Pose& pose);

// grasp candidates 采样函数声明
friend std::vector<geometry_msgs::msg::Pose> generateGraspCandidates(const geometry_msgs::msg::Pose& base_pose);

// 放置候选采样函数声明
friend std::vector<geometry_msgs::msg::Pose> generatePlaceCandidates(const geometry_msgs::msg::Pose& base_pose);

    // 补充声明：发布关节状态

    rclcpp::CallbackGroup::SharedPtr cb_group_;
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
    // 存储用于笛卡尔直线下降的最优轨迹（在 cpp 中被使用）
    moveit_msgs::msg::RobotTrajectory best_traj_;
    // 记录选定的预抓取位姿
    geometry_msgs::msg::Pose best_pre_grasp_pose_;
    
    struct TaskToken {};
    using TaskPtr = std::unique_ptr<TaskToken>;
    // 定义任务函数类型：接收一个 Token，返回一个 Token
    using OwnershipTask = std::function<TaskPtr(TaskPtr)>;

    bool busy_ = false;
};
