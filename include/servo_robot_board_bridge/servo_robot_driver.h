/*
 * servo_robot_driver C API — 手写头文件,与 src/ffi.rs 严格一致
 *
 * 构建:  cd cargo build --release --features ffi
 * 链接:  -L target/release -lservo_robot_driver
 *
 * 线程与生命周期红线
 * 1. 所有回调在驱动内部的分发线程上触发,C 侧回调必须线程安全。
 * 2. 回调内禁止调用任何 sr_driver_* 函数(尤其 sr_driver_free:
 *    drop 会 join 分发线程,而该线程正卡在你的回调里 = 自死锁)。
 * 3. 回调参数指针仅在回调执行期间有效,不要跨调用保存。
 * 4. 同步函数阻塞 ≤1s(驱动默认超时),不要在实时路径调用。
 * 5. 同一句柄的所有调用线程安全(内部有锁),但回调线程与调用线程并存。
 * 6. 禁止在仍有其他线程调用该句柄时调用 sr_driver_free(use-after-free);
 *    先 stop 并确保所有调用线程退出,再 free。
 * 7. GPL-3 许可:链接本库进入闭源程序有许可影响。
 */
#ifndef SERVO_ROBOT_DRIVER_H
#define SERVO_ROBOT_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// 不透明句柄
typedef struct sr_driver sr_driver;

/// 错误码(与 ffi.rs 常量一致;所有返回 int 的函数的返回值)
typedef enum {
    SR_OK = 0,
    SR_ERR_SERIAL = -1,
    SR_ERR_IO = -2,
    SR_ERR_FRAME = -3,
    SR_ERR_TRANSPORT_CLOSED = -4,
    SR_ERR_TIMEOUT = -5,
    SR_ERR_CRC = -6,
    SR_ERR_PAYLOAD_TOO_SHORT = -7,
    SR_ERR_UNKNOWN_FRAME = -8,
    SR_ERR_NOT_RUNNING = -9,
    SR_ERR_LOCK_POISONED = -10,
    SR_ERR_NULL = -11,
    SR_ERR_INVALID_ARG = -12,
    SR_ERR_PANIC = -13,
    SR_ERR_ALREADY_STARTED = -14,
} sr_error_code;

/// 配置类型(ConfigType,0x10~0x37;value 类型见各常量注释)
typedef enum {
    /// bool 开关:舵机电源
    SR_CONFIG_SWITCH_SERVO_POWER = 0x10,
    /// bool 开关:5V 电源
    SR_CONFIG_SWITCH_5V_POWER = 0x11,
    /// bool 开关:电池充电
    SR_CONFIG_SWITCH_CHARGE = 0x12,
    /// bool 开关:电池外部输出
    SR_CONFIG_SWITCH_BAT_EXT_OUT = 0x13,
    /// u8:充电容量上限百分比(1~100)
    SR_CONFIG_CHARGE_STOP_SOC = 0x20,
    /// u8:板子上报日志等级(SR_LOG_*)
    SR_CONFIG_TX_LOG_LEVEL = 0x21,
    /// u16:舵机电源限流(mA)
    SR_CONFIG_SERVO_CURRENT_LIMIT_MA = 0x30,
    /// u16:舵机电源温度上限(×10)
    SR_CONFIG_SERVO_TEMP_LIMIT = 0x31,
    /// u16:5V 电源温度上限(×10)
    SR_CONFIG_5V_TEMP_LIMIT = 0x32,
    /// u16:最大充电电流(mA)
    SR_CONFIG_CHARGE_MAX_CURRENT_MA = 0x33,
    /// u16:充电降流温度阈值(×10)
    SR_CONFIG_CHARGE_TEMP_DERATING = 0x34,
    /// u16:充电停止温度(×10)
    SR_CONFIG_CHARGE_TEMP_LIMIT = 0x35,
    /// u16:充电截止电压(mV)
    SR_CONFIG_CHARGE_STOP_VOLTAGE_MV = 0x36,
    /// u32:舵机串口波特率
    SR_CONFIG_SERVO_BAUD_RATE = 0x37,
} sr_config_type;

/// 板级命令(CommandType,一次性动作)
typedef enum {
    /// 重启 MCU
    SR_CMD_RESET = 0x01,
    /// 关机(切断全部电源)
    SR_CMD_SHUTDOWN = 0x02,
    /// 触发 OTA 更新(bootloader 拷贝 OTA Temp → App 后重启)
    SR_CMD_OTA = 0x03,
} sr_cmd_type;

/// 日志等级(板子 → PC)
typedef enum {
    SR_LOG_OFF = 0,
    SR_LOG_DEBUG = 1,
    SR_LOG_INFO = 2,
    SR_LOG_WARN = 3,
    SR_LOG_ERROR = 4,
} sr_log_level;

/// 电池充电状态
typedef enum {
    SR_BAT_CHARGE_UNKNOWN = 0,
    SR_BAT_CHARGE_CHARGING = 1,
    SR_BAT_CHARGE_DISCHARGING = 2,
    SR_BAT_CHARGE_NOT_CHARGING = 3,
    SR_BAT_CHARGE_FULL = 4,
} sr_battery_charge_status;

/// 电池健康状态
typedef enum {
    SR_BAT_HEALTH_UNKNOWN = 0,
    SR_BAT_HEALTH_GOOD = 1,
    SR_BAT_HEALTH_OVERHEAT = 2,
    SR_BAT_HEALTH_DEAD = 3,
    SR_BAT_HEALTH_OVERVOLTAGE = 4,
} sr_battery_health;

/// 电池技术类型
typedef enum {
    SR_BAT_TECH_UNKNOWN = 0,
    SR_BAT_TECH_NIMH = 1,
    SR_BAT_TECH_LION = 2,
    SR_BAT_TECH_LIPO = 3,
    SR_BAT_TECH_LIFE = 4,
    SR_BAT_TECH_NICD = 5,
    SR_BAT_TECH_LIMN = 6,
} sr_battery_technology;

/// 充电阶段
typedef enum {
    SR_CHARGE_PHASE_NOT_CHARGING = 0,
    SR_CHARGE_PHASE_PRE_CHARGE = 1,
    SR_CHARGE_PHASE_CC = 2,
    SR_CHARGE_PHASE_CV = 3,
    SR_CHARGE_PHASE_FULL = 4,
    SR_CHARGE_PHASE_PD_SINK_FAULT = 5,
    SR_CHARGE_PHASE_UNSUPPORTED_CHARGER = 6,
} sr_charge_phase;

/*  数据结构(字段顺序与 src/ffi.rs 严格一致)  */

/// 配置值:typ 为 SR_CONFIG_*,value 语义:bool→0/1,数值→原值
typedef struct {
    uint8_t typ;
    float value;
} sr_config;

/// 板级配置全量快照(24 字节 payload 的镜像)
typedef struct {
    uint8_t power_servo_on;
    uint8_t power_5v_on;
    uint8_t charge_on;
    uint8_t bat_ext_out_on;
    uint8_t charge_stop_percentage;
    uint8_t tx_log_level;
    uint16_t servo_current_limit_ma;
    uint16_t servo_temp_limit;
    uint16_t temp_5v_limit;
    uint16_t charge_max_current_ma;
    uint16_t charge_temp_derating;
    uint16_t charge_temp_limit;
    uint16_t charge_stop_voltage_mv;
    uint32_t servo_baud_rate;
} sr_board_config;

/// IMU 数据
typedef struct {
    /// 加速度计(m/s² 或 g,由固件决定)
    float accel[3];
    /// 陀螺仪(°/s)
    float gyro[3];
    /// 四元数(w, x, y, z)
    float quaternion[4];
    /// 固件时间戳(ms)
    uint32_t timestamp_ms;
    /// 欧拉角(°)
    float roll;
    float pitch;
    float yaw;
} sr_imu;

/// 电源数据(电压/电流字段:原始值 = 实际值 × 10)
typedef struct {
    /// 舵机电源输出电压
    uint16_t servo_voltage_mv;
    /// 舵机电源输出电流
    uint16_t servo_current_ma;
    /// 充电输入电压
    uint16_t charge_in_voltage_mv;
    /// 充电输入电流
    uint16_t charge_in_current_ma;
    /// 电池电压
    uint16_t bat_voltage_mv;
    /// 电池电流(+ 充电 / - 放电)
    int16_t bat_current_ma;
} sr_power;

/// 电池状态
typedef struct {
    /// 电池总电压(mV)
    uint16_t voltage_mv;
    /// 电池电流(+ 充电 / - 放电,mA)
    int16_t current_ma;
    /// 满充容量(mAh)
    uint16_t capacity_mah;
    /// 设计容量(mAh)
    uint16_t design_capacity_mah;
    /// 剩余电量百分比(1~100)
    uint8_t percentage;
    /// 整体温度(实际值 = 原始值 / 10)
    int16_t temperature;
    /// SR_BAT_CHARGE_*
    uint8_t charge_status;
    /// SR_BAT_HEALTH_*
    uint8_t health;
    /// SR_BAT_TECH_*
    uint8_t technology;
    /// 电池是否在位
    uint8_t present;
    /// 序列号
    uint16_t serial_number;
    /* 以下数组指针仅在回调执行期间有效 */
    /// 各电芯电压(mV),长度 cell_count
    const uint16_t* cell_voltages_mv;
    uint32_t cell_count;
    /// 各电芯温度(实际值 = 原始值 / 10),长度 cell_temp_count
    const int16_t* cell_temperatures;
    uint32_t cell_temp_count;
} sr_battery_state;

/// 板级事件
typedef struct {
    /// SR_CHARGE_PHASE_*
    uint8_t charge_phase;
    /// 状态变化位标志(见 README:充电器接入/风扇/舵机电源等)
    uint16_t state_change_flags;
    /// 保护位标志(过流/过温/低电量等)
    uint16_t protection_flags;
    /// 错误位标志(UART/I2C/SPI/USB 等)
    uint16_t error_flags;
} sr_board_event;

/// 系统信息(含温度数据)
typedef struct {
    /// STM32 设备 ID
    uint16_t device_id;
    /// STM32 全局 ID
    uint32_t uid;
    /// IMU 的 ID
    uint8_t imu_id;
    /// 运行时间(s)
    uint32_t uptime_s;
    /// CPU 占用率(%)
    uint8_t cpu_usage_percent;
    /// 空闲堆(KB)
    uint16_t free_heap_kb;
    /// 启动以来最小剩余栈空间(KB)
    uint16_t stack_watermark_min_kb;
    uint16_t i2c_error_count;
    uint16_t spi_error_count;
    uint16_t uart_error_count;
    uint16_t usb_error_count;
    /// 累计发送帧数
    uint32_t frames_sent_total;
    /// PD 握手请求电压(mV)
    uint16_t pd_request_voltage_mv;
    /// PD 握手请求电流(mA)
    uint16_t pd_request_current_ma;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    /* 温度字段:实际值 = 原始值 / 10 */
    /// 舵机电源温度
    int16_t temp_servo_power;
    /// 5V 电源温度
    int16_t temp_5v_power;
    /// MCU 温度
    int16_t temp_mcu;
    /// 充电电路温度
    int16_t temp_charge;
    /// 电池温度
    int16_t temp_battery;
} sr_system_info;

/// 板级日志消息
typedef struct {
    /// Unix 时间戳(毫秒),读线程解码时采集
    uint64_t ts_ms;
    /// SR_LOG_*
    uint8_t level;
    /* 以下字符串 NUL 结尾,仅在回调执行期间有效 */
    const char* file_name;
    const char* fun_name;
    const char* msg;
} sr_log_message;

/// 回调表(全 NULL 即可只注册部分;任意时刻可替换)
///
/// 所有回调在驱动分发线程上触发,第一个参数均为注册时传入的 userdata。
/// 回调内禁止调用任何 sr_driver_* 函数(见文件顶部红线)。
typedef struct {
    void* userdata;
    void (*on_imu_data)(void* userdata, const sr_imu* data);
    void (*on_power_data)(void* userdata, const sr_power* data);
    void (*on_battery_state)(void* userdata, const sr_battery_state* state);
    void (*on_config_snapshot)(void* userdata, const sr_board_config* config);
    void (*on_board_event)(void* userdata, const sr_board_event* event);
    void (*on_system_info)(void* userdata, const sr_system_info* info);
    void (*on_log)(void* userdata, const sr_log_message* msg);
    void (*on_ack_cfg_write)(void* userdata, uint8_t success);
    void (*on_ack_cfg_query)(void* userdata, const sr_config* config);
    void (*on_ack_cfg_query_all)(void* userdata, const sr_board_config* config);
    void (*on_ack_servo_cmd)(void* userdata, const uint8_t* data, size_t len);
    void (*on_ack_command)(void* userdata, uint8_t success);
    void (*on_ack_firmware_update)(void* userdata, uint8_t success, uint32_t offset);
    void (*on_error)(void* userdata, int error_code);
} sr_callbacks;

/*  生命周期  */

/// 打开串口并创建驱动句柄(不支持自动重连)。
///
/// @param port        串口设备路径,如 "/dev/ttyUSB0"、"/dev/ttyS0"(Windows: "COM3")
/// @param baud_rate   波特率,如 115200
/// @param err_buf     失败时写入错误描述(可为 NULL)
/// @param err_buf_len err_buf 容量;描述会被截断并 NUL 结尾
/// @return 驱动句柄;失败返回 NULL,err_buf 含错误描述
sr_driver* sr_driver_open(const char* port, uint32_t baud_rate,
                          char* err_buf, size_t err_buf_len);

/// 打开串口并创建支持自动重连的驱动句柄。
///
/// 断开后驱动按指数退避自动重连:第 n 次重试前等待
/// retry_interval_ms × backoff_multiplier^n(上限 max_retry_interval_ms)。
///
/// @param port                串口设备路径
/// @param baud_rate           波特率
/// @param max_retries         最大重试次数(0 = 不重连)
/// @param retry_interval_ms   首次重试等待(ms)
/// @param backoff_multiplier  退避倍数(须为有限非负数,非法返回 NULL)
/// @param max_retry_interval_ms 重试等待上限(ms)
/// @param err_buf / err_buf_len 同 sr_driver_open
/// @return 驱动句柄;失败返回 NULL,err_buf 含错误描述
sr_driver* sr_driver_open_reconnect(const char* port, uint32_t baud_rate,
                                    uint32_t max_retries, uint32_t retry_interval_ms,
                                    float backoff_multiplier, uint32_t max_retry_interval_ms,
                                    char* err_buf, size_t err_buf_len);

/// 释放句柄(内部 stop 并 join 读/分发线程)。NULL 安全。
///
/// 注意:调用前须确保没有其他线程正在使用该句柄(见文件顶部红线 6)。
/// @param d 句柄(可为 NULL)
void sr_driver_free(sr_driver* d);

/*  控制  */

/// 启动驱动(开启读线程 + 分发线程)。
///
/// @param d 句柄
/// @return SR_OK 成功;SR_ERR_ALREADY_STARTED 已启动;
///         SR_ERR_TRANSPORT_CLOSED 无可用传输层;其他见 sr_error_code
int sr_driver_start(sr_driver* d);

/// 停止驱动(置运行标志 false 并 join 读/分发线程)。
///
/// @param d 句柄
/// @return SR_OK 成功;其他见 sr_error_code
int sr_driver_stop(sr_driver* d);

/*  配置  */

/// 写入配置到板子(不等待应答)。
///
/// @param d   句柄
/// @param cfg 配置值(typ 为 SR_CONFIG_*)
/// @return SR_OK 已写入;SR_ERR_INVALID_ARG 未知配置类型;其他见 sr_error_code
int sr_driver_write_config(sr_driver* d, sr_config cfg);

/// 写入配置并等待板子确认(阻塞 ≤1s)。
///
/// @param d           句柄
/// @param cfg         配置值
/// @param out_success [out] 非空时写入板子 ACK 结果(1 = 接受,0 = 拒绝)
/// @return SR_OK 收到应答;SR_ERR_TIMEOUT 超时;其他见 sr_error_code
int sr_driver_write_config_sync(sr_driver* d, sr_config cfg, uint8_t* out_success);

/// 查询单个配置并等待响应(阻塞 ≤1s)。
///
/// @param d   句柄
/// @param typ SR_CONFIG_* 配置类型
/// @param out [out] 查询结果(typ 回显 + value)
/// @return SR_OK 成功;SR_ERR_TIMEOUT 超时;SR_ERR_INVALID_ARG 未知类型;其他见 sr_error_code
int sr_driver_query_config(sr_driver* d, uint8_t typ, sr_config* out);

/// 查询全部配置并等待响应(阻塞 ≤1s)。
///
/// @param d   句柄
/// @param out [out] 24 字节快照(14 个字段)
/// @return SR_OK 成功;SR_ERR_TIMEOUT 超时;其他见 sr_error_code
int sr_driver_query_all_configs(sr_driver* d, sr_board_config* out);

/*  舵机(透传原始舵机命令字节,内容取决于舵机协议)  */

/// 转发舵机命令(不等待应答)。
///
/// @param d    句柄
/// @param data 原始舵机命令字节
/// @param len  data 长度
/// @return SR_OK 已写入;其他见 sr_error_code
int sr_driver_forward_servo(sr_driver* d, const uint8_t* data, size_t len);

/// 转发舵机命令并等待板子应答(阻塞 ≤1s)。
///
/// @param d       句柄
/// @param data    原始舵机命令字节
/// @param len     data 长度
/// @param out     [out] 应答缓冲区(调用方分配)
/// @param cap     out 容量
/// @param out_len [out] 应答实际长度;缓冲区不足时写入所需长度并返回
///                SR_ERR_PAYLOAD_TOO_SHORT,可据此扩容重试
/// @return SR_OK 成功;SR_ERR_TIMEOUT 超时;SR_ERR_PAYLOAD_TOO_SHORT 缓冲不足;
///         其他见 sr_error_code
int sr_driver_forward_servo_sync(sr_driver* d, const uint8_t* data, size_t len,
                                 uint8_t* out, size_t cap, size_t* out_len);

/*  板级命令(SR_CMD_*)  */

/// 发送一次性板级命令(不等待应答)。
///
/// @param d   句柄
/// @param cmd SR_CMD_*(0x01 Reset / 0x02 Shutdown / 0x03 Ota)
/// @return SR_OK 已写入;SR_ERR_INVALID_ARG 未知命令;其他见 sr_error_code
int sr_driver_send_command(sr_driver* d, uint8_t cmd);

/// 发送一次性板级命令并等待确认(阻塞 ≤1s)。
///
/// @param d          句柄
/// @param cmd        SR_CMD_*
/// @param out_success [out] 非空时写入板子 ACK 结果(1 = 执行成功)
/// @return SR_OK 收到应答;SR_ERR_TIMEOUT 超时;其他见 sr_error_code
int sr_driver_send_command_sync(sr_driver* d, uint8_t cmd, uint8_t* out_success);

/*  固件更新  */

/// 发送固件更新数据块(不等待应答)。
///
/// @param d      句柄
/// @param offset 固件写入偏移(字节)
/// @param data   数据块字节
/// @param len    data 长度
/// @return SR_OK 已写入;其他见 sr_error_code
int sr_driver_firmware_update(sr_driver* d, uint32_t offset, const uint8_t* data, size_t len);

/// 发送固件更新数据块并等待确认(阻塞 ≤1s)。
///
/// @param d           句柄
/// @param offset      固件写入偏移(字节)
/// @param data        数据块字节
/// @param len         data 长度
/// @param out_success [out] 非空时写入板子 ACK 结果(1 = 接受该块)
/// @return SR_OK 收到应答;SR_ERR_TIMEOUT 超时;其他见 sr_error_code
int sr_driver_firmware_update_sync(sr_driver* d, uint32_t offset, const uint8_t* data,
                                   size_t len, uint8_t* out_success);

/*  回调与诊断  */

/// 设置/替换 C 回调表(任意时刻可调用;NULL 槽位被忽略)。
///
/// 回调在驱动分发线程上触发,不受本调用所在线程影响。
///
/// @param d   句柄
/// @param cbs 回调表(可为全 NULL 的表)
/// @return SR_OK 成功;其他见 sr_error_code
int sr_driver_set_callbacks(sr_driver* d, const sr_callbacks* cbs);

/// 拷贝最近一次驱动错误的描述(截断 + NUL 结尾)。
///
/// @param d   句柄
/// @param buf [out] 输出缓冲区
/// @param len buf 容量
/// @return SR_OK 成功;其他见 sr_error_code
int sr_driver_last_error(sr_driver* d, char* buf, size_t len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SERVO_ROBOT_DRIVER_H */
