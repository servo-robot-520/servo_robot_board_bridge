//
// Created by greenhand520 on 2026/8/15.
//

#include "servo_robot_board_bridge/bridge.hpp"

#include <format>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp_action/create_server.hpp>

#include "common_cpp/logging/log.hpp"
#include "common_cpp/logging/log_interface/log_interface.hpp"
#include "common_cpp/logging/log_interface/log_manager.hpp"

using namespace std::chrono_literals;
using namespace servo_robot_board_interface;

namespace srbb {

    /* extern "C" thunks — 转发 FFI 回调到 CallbackCtx */
    extern "C" {
    static void power_thunk(void* u, const sr_power* d) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_power)
            c->on_power(d);
    }
    static void event_thunk(void* u, const sr_board_event* d) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_event)
            c->on_event(d);
    }
    static void config_thunk(void* u, const sr_board_config* d) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_config)
            c->on_config(d);
    }
    static void log_thunk(void* u, const sr_log_message* d) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_log)
            c->on_log(d);
    }
    static void system_thunk(void* u, const sr_system_info* d) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_system)
            c->on_system(d);
    }
    static void error_thunk(void* u, const int code) {
        if (const auto* c = static_cast<DriverCallbackCtx*>(u); c && c->on_error)
            c->on_error(code);
    }
    } // extern "C"


    /// Helper: sr_driver error → string
    static const char* sr_err_name(const int rc) {
        // sr_error_code: 0 (OK) .. -14 (ALREADY_STARTED) 连续
        static constexpr const char* kNames[] = {
            "OK",
            "serial error",
            "IO error",
            "frame parse error",
            "transport closed",
            "timeout",
            "CRC mismatch",
            "payload too short",
            "unknown frame",
            "not running",
            "lock poisoned",
            "null pointer",
            "invalid argument",
            "driver panic",
            "already started",
        };
        if (rc >= SR_ERR_ALREADY_STARTED && rc <= 0) {
            return kNames[-rc];
        }
        return "unknown error";
    }


    Bridge::Bridge(rclcpp_lifecycle::LifecycleNode& node) : node_(&node), logger_(node.get_logger()) {

        const auto port = node_->declare_parameter<std::string>("serial.port", "/dev/ttyUSB0");
        const auto baud = static_cast<uint32_t>(node_->declare_parameter<int>("serial.baud_rate", 115200));
        const auto max_retries = static_cast<uint32_t>(node_->declare_parameter<int>("reconnect.max_retries", 10));
        const auto retry_interval = static_cast<uint32_t>(node_->declare_parameter<int>("reconnect.retry_interval_ms", 2000));
        const auto backoff = static_cast<float>(node_->declare_parameter<double>("reconnect.backoff_multiplier", 1.5));
        const auto max_retry_interval = static_cast<uint32_t>(node_->declare_parameter<int>("reconnect.max_retry_interval_ms", 30000));

        char err_buf[256];
        driver_ = sr_driver_open_reconnect(port.c_str(), baud, max_retries, retry_interval, backoff, max_retry_interval,
                                           err_buf, sizeof err_buf);
        if (!driver_) {
            throw std::runtime_error(std::format("sr_driver_open_reconnect({}) failed: {}", port, err_buf));
        }

        RCLCPP_INFO(logger_, "Driver opened: %s @ %u baud", port.c_str(), baud);

        cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        setup_publishers();
        setup_callbacks();
        setup_services();
        setup_action_server();

        if (const int rc = sr_driver_start(driver_); rc != SR_OK) {
            sr_driver_free(driver_);
            driver_ = nullptr;
            throw std::runtime_error(
                std::format("sr_driver_start failed: {}", sr_err_name(rc)));
        }

        RCLCPP_INFO(logger_, "Driver started");
    }

    Bridge::~Bridge() {
        if (driver_) {
            sr_driver_stop(driver_);
            sr_driver_free(driver_);
            driver_ = nullptr;
            RCLCPP_INFO(logger_, "Driver stopped and freed");
        }
    }


    void Bridge::setup_publishers() {
        power_publisher_ = node_->create_publisher<MsgPower>(std::string{kPrivateTopicBoardPower}, rclcpp::QoS(10));
        event_publisher_ = node_->create_publisher<MsgEvent>(std::string{kPrivateTopicBoardEvent}, rclcpp::QoS(10));
        config_publisher_ = node_->create_publisher<MsgConfig>(std::string{kPrivateTopicBoardConfig}, rclcpp::QoS(10));
        diagnostics_publisher_ = node_->create_publisher<DiagnosticStatus>("/diagnostics", rclcpp::QoS(10));
    }

    // ═══ Callbacks (驱动分发线程执行, 禁止调用 sr_driver_*) ════════

    void Bridge::setup_callbacks() {
        // Power → MsgPower
        driver_ctx_.on_power = [this](const sr_power* d) {
            RCLCPP_DEBUG(logger_, "on_power: servo=%dmV/%dmA bat=%dmV/%dmA",
                         d->servo_voltage_mv, d->servo_current_ma,
                         d->bat_voltage_mv, d->bat_current_ma);
            MsgPower msg;
            msg.servo_voltage_mv = d->servo_voltage_mv;
            msg.servo_current_ma = d->servo_current_ma;
            msg.charge_in_voltage_mv = d->charge_in_voltage_mv;
            msg.charge_in_current_ma = d->charge_in_current_ma;
            msg.bat_voltage_mv = d->bat_voltage_mv;
            msg.bat_current_ma = d->bat_current_ma;
            power_publisher_->publish(msg);
        };

        // Event → MsgEvent
        driver_ctx_.on_event = [this](const sr_board_event* d) {
            RCLCPP_DEBUG(logger_, "on_event: charge=%u state=%u prot=%u err=%u",
                         d->charge_phase, d->state_change_flags,
                         d->protection_flags, d->error_flags);
            MsgEvent msg;
            msg.charge_phase = d->charge_phase;
            msg.state_change_flags = d->state_change_flags;
            msg.protection_flags = d->protection_flags;
            msg.error_flags = d->error_flags;
            event_publisher_->publish(msg);
        };

        // Config snapshot → MsgConfig
        driver_ctx_.on_config = [this](const sr_board_config* d) {
            RCLCPP_DEBUG(logger_, "on_config: servo=%d 5v=%d charge=%d bat_ext=%d",
                         d->power_servo_on, d->power_5v_on, d->charge_on, d->bat_ext_out_on);
            MsgConfig msg;
            msg.power_servo_on = d->power_servo_on;
            msg.power_5v_on = d->power_5v_on;
            msg.charge_on = d->charge_on;
            msg.bat_ext_out_on = d->bat_ext_out_on;
            msg.charge_stop_percentage = d->charge_stop_percentage;
            msg.tx_log_level = d->tx_log_level;
            msg.servo_current_limit_ma = d->servo_current_limit_ma;
            msg.servo_temp_limit = static_cast<float>(d->servo_temp_limit);
            msg.temp_5v_limit = static_cast<float>(d->temp_5v_limit);
            msg.charge_max_current_ma = d->charge_max_current_ma;
            msg.charge_temp_derating = static_cast<float>(d->charge_temp_derating);
            msg.charge_temp_limit = static_cast<float>(d->charge_temp_limit);
            msg.charge_stop_voltage_mv = d->charge_stop_voltage_mv;
            msg.servo_baud_rate = d->servo_baud_rate;
            config_publisher_->publish(msg);
        };

        // Log
        driver_ctx_.on_log = [this](const sr_log_message* d) {
            RCLCPP_DEBUG(logger_, "on_log: level=%u", d->level);
            const auto logger = log_interface::LogManager::get();
            if (logger) {
                logger->log(static_cast<log_interface::LogInterface::Level>(d->level), d->file_name, -1, d->fun_name, d->msg);
            }
        };

        // System info → MsgSystem
        driver_ctx_.on_system = [this](const sr_system_info* d) {
            RCLCPP_DEBUG(logger_, "on_system: id=%hu uptime=%us cpu=%u%% heap=%ukB",
                         d->device_id, d->uptime_s, d->cpu_usage_percent, d->free_heap_kb);
            MsgSystem msg;
            msg.device_id = d->device_id;
            msg.uid = d->uid;
            msg.imu_id = d->imu_id;
            msg.uptime_s = d->uptime_s;
            msg.cpu_usage_percent = d->cpu_usage_percent;
            msg.free_heap_kb = d->free_heap_kb;
            msg.stack_watermark_min_kb = d->stack_watermark_min_kb;
            msg.i2c_error_count = d->i2c_error_count;
            msg.spi_error_count = d->spi_error_count;
            msg.uart_error_count = d->uart_error_count;
            msg.usb_error_count = d->usb_error_count;
            msg.frames_sent_total = d->frames_sent_total;
            msg.pd_request_voltage_mv = d->pd_request_voltage_mv;
            msg.pd_request_current_ma = d->pd_request_current_ma;
            // 原始值 / 10 = 实际温度
            msg.temp_servo_power = static_cast<float>(d->temp_servo_power) / 10.0f;
            msg.temp_5v_power = static_cast<float>(d->temp_5v_power) / 10.0f;
            msg.temp_mcu = static_cast<float>(d->temp_mcu) / 10.0f;
            msg.temp_charge = static_cast<float>(d->temp_charge) / 10.0f;
            msg.temp_battery = static_cast<float>(d->temp_battery) / 10.0f;
            // firmware version: major.minor.patch
            msg.firmware_version = std::format("{}.{}.{}", d->fw_major, d->fw_minor, d->fw_patch);
            // todo：输出到诊断话题
        };

        // Error
        driver_ctx_.on_error = [this](const int code) {
            RCLCPP_ERROR(logger_, "Driver error: %s (%d)", sr_err_name(code), code);
        };

        // Register with driver
        sr_callbacks cbs{};
        cbs.userdata = &driver_ctx_;
        cbs.on_power_data = power_thunk;
        cbs.on_board_event = event_thunk;
        cbs.on_config_snapshot = config_thunk;
        cbs.on_log = log_thunk;
        cbs.on_system_info = system_thunk;
        cbs.on_error = error_thunk;

        if (const int rc = sr_driver_set_callbacks(driver_, &cbs); rc != SR_OK) {
            RCLCPP_WARN(logger_, "set_callbacks failed: %s", sr_err_name(rc));
        }
    }


    void Bridge::handle_query_config(const std::shared_ptr<SrvQueryConfig::Request>& request,
                                     const std::shared_ptr<SrvQueryConfig::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv query_config: type=%u", request->config_type);
        sr_config out{};
        if (const int rc = sr_driver_query_config(driver_, request->config_type, &out); rc == SR_OK) {
            response->success = true;
            response->value = out.value;
            response->msg = "ok";
        }
        else {
            response->success = false;
            response->value = 0.0f;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "query_config failed: type=%u err=%s", request->config_type, response->msg.c_str());
        }
    }

    void Bridge::handle_query_all_config(const std::shared_ptr<SrvQueryAllConfig::Request>&,
                                         const std::shared_ptr<SrvQueryAllConfig::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv query_all_config");
        sr_board_config out{};
        if (const int rc = sr_driver_query_all_configs(driver_, &out); rc == SR_OK) {
            response->success = true;
            response->msg = "ok";
            auto& cfg = response->config;
            cfg.power_servo_on = out.power_servo_on;
            cfg.power_5v_on = out.power_5v_on;
            cfg.charge_on = out.charge_on;
            cfg.bat_ext_out_on = out.bat_ext_out_on;
            cfg.charge_stop_percentage = out.charge_stop_percentage;
            cfg.tx_log_level = out.tx_log_level;
            cfg.servo_current_limit_ma = out.servo_current_limit_ma;
            cfg.servo_temp_limit = static_cast<float>(out.servo_temp_limit);
            cfg.temp_5v_limit = static_cast<float>(out.temp_5v_limit);
            cfg.charge_max_current_ma = out.charge_max_current_ma;
            cfg.charge_temp_derating = static_cast<float>(out.charge_temp_derating);
            cfg.charge_temp_limit = static_cast<float>(out.charge_temp_limit);
            cfg.charge_stop_voltage_mv = out.charge_stop_voltage_mv;
            cfg.servo_baud_rate = out.servo_baud_rate;
        }
        else {
            response->success = false;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "query_all_config failed: %s", response->msg.c_str());
        }
    }

    void Bridge::handle_write_config(const std::shared_ptr<SrvWriteConfig::Request>& request,
                                     const std::shared_ptr<SrvWriteConfig::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv write_config: type=%u value=%.2f", request->config_type, request->value);
        sr_config cfg{};
        cfg.typ = request->config_type;
        cfg.value = request->value;
        uint8_t ack = 0;
        if (const int rc = sr_driver_write_config_sync(driver_, cfg, &ack); rc == SR_OK) {
            response->success = ack != 0;
            response->msg = ack ? "ACK" : "NACK";
            if (!response->success) {
                RCLCPP_ERROR(logger_, "write_config NACK: type=%u value=%.2f", request->config_type, request->value);
            }
        }
        else {
            response->success = false;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "write_config failed: type=%u err=%s", request->config_type, response->msg.c_str());
        }
    }

    void Bridge::handle_switch(const std::shared_ptr<SrvSwitch::Request>& request,
                               const std::shared_ptr<SrvSwitch::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv switch: type=%u enable=%d", request->switch_type, request->enable);
        sr_config cfg{};
        cfg.typ = request->switch_type;
        cfg.value = request->enable ? 1.0f : 0.0f;
        uint8_t ack = 0;
        if (const int rc = sr_driver_write_config_sync(driver_, cfg, &ack); rc == SR_OK) {
            response->success = ack != 0;
            response->msg = ack ? "ACK" : "NACK";
            if (!response->success) {
                RCLCPP_ERROR(logger_, "switch NACK: type=%u enable=%d", request->switch_type, request->enable);
            }
        }
        else {
            response->success = false;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "switch failed: type=%u err=%s", request->switch_type, response->msg.c_str());
        }
    }

    void Bridge::handle_command(const std::shared_ptr<SrvCommand::Request>& request,
                                const std::shared_ptr<SrvCommand::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv command: type=%u", request->command_type);
        uint8_t ack = 0;
        if (const int rc = sr_driver_send_command_sync(driver_, request->command_type, &ack); rc == SR_OK) {
            response->success = ack != 0;
            response->msg = ack ? "ACK" : "NACK";
            if (!response->success) {
                RCLCPP_ERROR(logger_, "command NACK: type=%u", request->command_type);
            }
        }
        else {
            response->success = false;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "command failed: type=%u err=%s", request->command_type, response->msg.c_str());
        }
    }

    void Bridge::handle_servo_forward(const std::shared_ptr<SrvServoForward::Request>& request,
                                      const std::shared_ptr<SrvServoForward::Response>& response) const {
        RCLCPP_DEBUG(logger_, "srv servo_forward: cmd_len=%zu", request->command.size());
        uint8_t buf[512]{};
        size_t out_len = 0;
        const int rc = sr_driver_forward_servo_sync(driver_, request->command.data(), request->command.size(),
                                                    buf, sizeof buf, &out_len);
        if (rc == SR_OK) {
            response->success = true;
            response->response.assign(buf, buf + out_len);
            response->msg = "ok";
        }
        else {
            response->success = false;
            response->msg = sr_err_name(rc);
            RCLCPP_ERROR(logger_, "servo_forward failed: cmd_len=%zu err=%s", request->command.size(), response->msg.c_str());
        }
    }

    void Bridge::setup_services() {

        query_cfg_service_ = node_->create_service<SrvQueryConfig>(
            std::string{kPrivateSrvQueryConfig},
            [this](const std::shared_ptr<SrvQueryConfig::Request>& request,
                   const std::shared_ptr<SrvQueryConfig::Response>& response) {
                handle_query_config(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);

        query_all_cfg_service_ = node_->create_service<SrvQueryAllConfig>(
            std::string{kPrivateSrvQueryAllConfig},
            [this](const std::shared_ptr<SrvQueryAllConfig::Request>& request,
                   const std::shared_ptr<SrvQueryAllConfig::Response>& response) {
                handle_query_all_config(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);

        write_cfg_service_ = node_->create_service<SrvWriteConfig>(
            std::string{kPrivateSrvWriteConfig},
            [this](const std::shared_ptr<SrvWriteConfig::Request>& request,
                   const std::shared_ptr<SrvWriteConfig::Response>& response) {
                handle_write_config(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);

        switch_service_ = node_->create_service<SrvSwitch>(
            std::string{kPrivateSrvSwitch},
            [this](const std::shared_ptr<SrvSwitch::Request>& request,
                   const std::shared_ptr<SrvSwitch::Response>& response) {
                handle_switch(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);

        cmd_service_ = node_->create_service<SrvCommand>(
            std::string{kPrivateSrvServoCommand},
            [this](const std::shared_ptr<SrvCommand::Request>& request,
                   const std::shared_ptr<SrvCommand::Response>& response) {
                handle_command(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);

        servo_forward_service_ = node_->create_service<SrvServoForward>(
            std::string{kPrivateSrvServoForward},
            [this](const std::shared_ptr<SrvServoForward::Request>& request,
                   const std::shared_ptr<SrvServoForward::Response>& response) {
                handle_servo_forward(request, response);
            },
            rmw_qos_profile_services_default, cb_group_);
    }


    rclcpp_action::GoalResponse Bridge::handle_firmware_update_goal(const rclcpp_action::GoalUUID&,
                                                                    const std::shared_ptr<const ActionFirmwareUpdate::Goal>& goal) {
        RCLCPP_DEBUG(logger_, "FirmwareUpdate: goal received offset=%u size=%zu", goal->offset, goal->data.size());
        if (goal->data.empty()) {
            RCLCPP_WARN(logger_, "FirmwareUpdate: rejecting empty data");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse Bridge::handle_firmware_update_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>&) {
        RCLCPP_DEBUG(logger_, "FirmwareUpdate: cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Bridge::handle_firmware_update_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& goal_handle) {
        RCLCPP_DEBUG(logger_, "FirmwareUpdate: goal accepted, spawning execution thread");
        std::thread{&Bridge::execute_firmware_update, this, goal_handle}.detach();
    }

    void Bridge::execute_firmware_update(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& goal_handle) {
        const auto goal = goal_handle->get_goal();
        const auto feedback = std::make_shared<ActionFirmwareUpdate::Feedback>();
        const auto result = std::make_shared<ActionFirmwareUpdate::Result>();

        RCLCPP_INFO(logger_, "FirmwareUpdate: sending %zu bytes at offset %u", goal->data.size(), goal->offset);

        uint8_t ack = 0;
        const int rc = sr_driver_firmware_update_sync(
            driver_,
            goal->offset,
            goal->data.data(), goal->data.size(),
            &ack);

        if (rc == SR_OK && ack) {
            feedback->progress = 1.0f;
            goal_handle->publish_feedback(feedback);
            result->success = true;
            result->msg = "ok";
            RCLCPP_INFO(logger_, "FirmwareUpdate: chunk accepted");
        }
        else {
            result->success = false;
            result->msg = rc != SR_OK ? sr_err_name(rc) : "NACK from board";
            RCLCPP_ERROR(logger_, "FirmwareUpdate: failed: %s", result->msg.c_str());
        }

        goal_handle->succeed(result);
    }

    void Bridge::setup_action_server() {
        firmware_update_server_ = rclcpp_action::create_server<ActionFirmwareUpdate>(
            node_->get_node_base_interface(),
            node_->get_node_clock_interface(),
            node_->get_node_logging_interface(),
            node_->get_node_waitables_interface(),
            std::string{kPrivateActionFirmwareUpdate},
            [this](const rclcpp_action::GoalUUID& uuid,
                   const std::shared_ptr<const ActionFirmwareUpdate::Goal>& goal) {
                return handle_firmware_update_goal(uuid, goal);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& h) {
                return handle_firmware_update_cancel(h);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionFirmwareUpdate>>& h) {
                handle_firmware_update_accepted(h);
            },
            rcl_action_server_get_default_options(), cb_group_);
    }

} // namespace srbb
