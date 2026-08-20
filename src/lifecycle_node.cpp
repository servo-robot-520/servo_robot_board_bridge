//
// Created by greenhand520 on 2026/8/18.
//

#include "servo_robot_board_bridge/lifecycle_node.hpp"

namespace srbb {

    BridgeLifecycleNode::~BridgeLifecycleNode() = default;

    CallbackReturn BridgeLifecycleNode::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Configuring...");
        try {
            bridge_ = std::make_unique<Bridge>(*this);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to create bridge: %s", e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "Configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BridgeLifecycleNode::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        bridge_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BridgeLifecycleNode::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        bridge_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BridgeLifecycleNode::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_ERROR(get_logger(), "Error occurred");
        bridge_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BridgeLifecycleNode::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BridgeLifecycleNode::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Deactivated");
        return CallbackReturn::SUCCESS;
    }

} // namespace srbb
