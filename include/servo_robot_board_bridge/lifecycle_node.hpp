#pragma once

#include <memory>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "servo_robot_board_bridge/bridge.hpp"


namespace srbb {

    class BridgeLifecycleNode : public rclcpp_lifecycle::LifecycleNode {

        std::unique_ptr<Bridge> bridge_;

    public:
        BridgeLifecycleNode(const std::string& node_name, const rclcpp::NodeOptions& options)
            : LifecycleNode(node_name, options) {
        }

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        ~BridgeLifecycleNode() override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    };


} // namespace srbb
