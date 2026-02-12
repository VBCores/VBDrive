#pragma once

#include "main.h"
#include "stm32g4xx_hal.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/config/serial/serial.h>

struct VBDriveConfig: public BaseConfigData {
    static constexpr uint32_t TYPE_ID = 0x44CCCBBB;
    uint8_t gear_ratio = 0;
    // NAN means not set
    float max_voltage = NAN;
    float max_current = NAN;
    float max_torque = NAN;
    float max_speed = NAN;
    float angle_offset = NAN;
    float min_angle = NAN;
    float max_angle = NAN;
    float torque_const = NAN;
    float kp = NAN;
    float ki = NAN;
    float kd = NAN;
    float filter_a = NAN;
    float filter_g1 = NAN;
    float filter_g2 = NAN;
    float filter_g3 = NAN;

    VBDriveConfig() {
        type_id = VBDriveConfig::TYPE_ID;
        gear_ratio = 1;
        filter_a = 0.0f;
    }

    bool are_required_params_set();

    void print_self(UARTResponseAccumulator& responses);
    void get(const std::string& param, UARTResponseAccumulator& responses);
    bool set(const std::string& param, std::string& value, UARTResponseAccumulator& responses);
};

constexpr size_t CALIBRATION_PLACEMENT = 0;
constexpr size_t CONFIG_PLACEMENT = CALIBRATION_PLACEMENT + sizeof(CalibrationData) + 1;
constexpr size_t IND_SENSOR_STATE_PLACEMENT = CONFIG_PLACEMENT + sizeof(VBDriveConfig) + 1;

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 40;

// NOTE: due to RAM constraints, this buffer if first used for calibration then reused for Cyphal queue.
//       It should be big enough for both purposes. For calibration, it should be >=(1024+1)*4 for guaranteed alignment
constexpr size_t SHARED_BUFFER_SIZE = std::max(
    (CALIBRATION_BUFF_SIZE + 1) * sizeof(int),
    static_cast<size_t>(CYPHAL_QUEUE_SIZE * sizeof(CanardTxQueueItem) * QUEUE_SIZE_MULT)
);
extern std::byte cyphal_queue_buffer_shared[SHARED_BUFFER_SIZE];

inline constexpr millis DELAY_ON_ERROR_MS = 500;
std::shared_ptr<CyphalInterface> get_interface();
void in_loop_reporting(millis);
void setup_subscriptions();
void cyphal_loop();
void start_cyphal();
void set_cyphal_mode(uint8_t mode);

// common.cpp
micros system_time();
micros micros_64();
millis millis_32();
void start_timers();

// app.cpp
VBDrive* get_motor();
EEPROM& get_eeprom();

//fdcan.cpp
extern FDCAN_HandleTypeDef hfdcan1;

// config.cpp
using AppConfigT = AppConfigurator<VBDriveConfig, CONFIG_PLACEMENT>;
AppConfigT& get_app_config();
void configure_fdcan(FDCAN_HandleTypeDef*);

template <typename T>
inline T value_or_default(T value, T default_value) {
    return std::isnan(value) ? default_value : value;
}

template <typename T>
inline T value_or_default(T value, T default_value, T not_set_value) {
    return (value == not_set_value) ? default_value : value;
}
