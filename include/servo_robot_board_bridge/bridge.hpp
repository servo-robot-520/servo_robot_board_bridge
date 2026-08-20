#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "servo_robot_board_interface/ros_msg.hpp"
#include "servo_robot_driver.h"


namespace srbb {

    using namespace servo_robot_board_interface;
    using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

    /// @brief 回调桥: FFI 需要 C 链接的裸函数指针, 类方法不能直接进 sr_callbacks。
    /// userdata 指向此结构, extern "C" thunk 转发到 std::function 槽位。
    struct DriverCallbackCtx {
        std::function<void(const sr_power*)> on_power;
        std::function<void(const sr_board_event*)> on_event;
        std::function<void(const sr_board_config*)> on_config;
        std::function<void(const sr_log_message*)> on_log;
        std::function<void(const sr_system_info*)> on_system;
        std::function<void(int)> on_error;
    };

    class Bridge {
        rclcpp_lifecycle::LifecycleNode* node_;
        rclcpp::Logger logger_;

        /// Driver (ctx_ 先于 driver_: 析构时 driver_ 先销毁, 回调仍安全)
        DriverCallbackCtx driver_ctx_;
        sr_driver* driver_ = nullptr;

        rclcpp::Publisher<MsgPower>::SharedPtr power_publisher_;
        rclcpp::Publisher<MsgEvent>::SharedPtr event_publisher_;
        rclcpp::Publisher<MsgConfig>::SharedPtr config_publisher_;
        rclcpp::Publisher<DiagnosticStatus>::SharedPtr diagnostics_publisher_;

        rclcpp::Service<SrvQueryConfig>::SharedPtr query_cfg_service_;
        rclcpp::Service<SrvQueryAllConfig>::SharedPtr query_all_cfg_service_;
        rclcpp::Service<SrvWriteConfig>::SharedPtr write_cfg_service_;
        rclcpp::Service<SrvSwitch>::SharedPtr switch_service_;
        rclcpp::Service<SrvCommand>::SharedPtr cmd_service_;
        rclcpp::Service<SrvServoForward>::SharedPtr servo_forward_service_;

        rclcpp_action::Server<ActionFirmwareUpdate>::SharedPtr firmware_update_server_;

        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp::TimerBase::SharedPtr publish_timer_;

    public:
        explicit Bridge(rclcpp_lifecycle::LifecycleNode& node);
        ~Bridge();

        Bridge(const Bridge&) = delete;
        Bridge& operator=(const Bridge&) = delete;

    private:
        void setup_publishers();

        void setup_services();

        void setup_action_server();

        void setup_callbacks();

        void handle_query_config(const std::shared_ptr<SrvQueryConfig::Request>& request,
                                 const std::shared_ptr<SrvQueryConfig::Response>& response) const;

        void handle_query_all_config(const std::shared_ptr<SrvQueryAllConfig::Request>& request,
                                     const std::shared_ptr<SrvQueryAllConfig::Response>& response) const;

        void handle_write_config(const std::shared_ptr<SrvWriteConfig::Request>& request,
                                 const std::shared_ptr<SrvWriteConfig::Response>& response) const;

        void handle_switch(const std::shared_ptr<SrvSwitch::Request>& request,
                           const std::shared_ptr<SrvSwitch::Response>& response) const;

        void handle_command(const std::shared_ptr<SrvCommand::Request>& request,
                            const std::shared_ptr<SrvCommand::Response>& response) const;

        void handle_servo_forward(const std::shared_ptr<SrvServoForward::Request>& request,
                                  const std::shared_ptr<SrvServoForward::Response>& response) const;

        rclcpp_action::GoalResponse handle_firmware_update_goal(const rclcpp_action::GoalUUID& uuid,
                                                                const std::shared_ptr<const ActionFirmwareUpdate::Goal>& goal);

        rclcpp_action::CancelResponse handle_firmware_update_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& goal_handle);

        void handle_firmware_update_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& goal_handle);

        void execute_firmware_update(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& goal_handle);
    };


} // namespace srbb
