//
// Created by greenhand520 on 2026/8/15.
//

#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "servo_robot_board_bridge/lifecycle_node.hpp"


int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<srbb::BridgeLifecycleNode>(
        "servo_robot_board_bridge", rclcpp::NodeOptions{});

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
