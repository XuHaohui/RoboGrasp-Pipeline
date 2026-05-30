#include "piper_highlevel/gripper_force_ctrl.hpp"

#include <thread>

namespace piper_highlevel {

GripperForceController::GripperForceController(float threshold)
    : threshold_(threshold)
{
}

void GripperForceController::attach(rclcpp::Node& node)
{
    constexpr int kQueueSize = 10;
    sub_ = node.create_subscription<std_msgs::msg::Float32>(
        "/mujoco/gripper_contact", kQueueSize,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            contactCallback(msg);
        });
}

bool GripperForceController::isInContact() const
{
    return force_.load(std::memory_order_acquire) > threshold_;
}

float GripperForceController::getContactForce() const
{
    return force_.load(std::memory_order_acquire);
}

bool GripperForceController::waitForContact(std::chrono::milliseconds timeout)
{
    const auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < timeout) {
        if (isInContact()) {
            return true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    return false;
}

void GripperForceController::contactCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    force_.store(msg->data, std::memory_order_release);
    std::lock_guard<std::mutex> lock(mutex_);
    last_update_ = rclcpp::Clock().now();
}

}  // namespace piper_highlevel
