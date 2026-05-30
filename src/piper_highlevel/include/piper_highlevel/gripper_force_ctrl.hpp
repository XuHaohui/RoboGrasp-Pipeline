#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace piper_highlevel {

constexpr float kGripperContactForceThreshold = 50.0f;
constexpr auto kGripperContactTimeout = std::chrono::milliseconds(3000);

class GripperForceController {
public:
    explicit GripperForceController(float threshold = kGripperContactForceThreshold);

    void attach(rclcpp::Node& node);

    bool isInContact() const;

    float getContactForce() const;

    bool waitForContact(std::chrono::milliseconds timeout = kGripperContactTimeout);

private:
    void contactCallback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    std::atomic<float> force_{0.0f};
    float threshold_;
    mutable std::mutex mutex_;
    rclcpp::Time last_update_;
};

}  // namespace piper_highlevel
