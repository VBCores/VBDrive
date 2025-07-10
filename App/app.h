#pragma once

#include "main.h"
#include "stm32g4xx_hal.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/config/serial/serial.h>

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 50;
inline constexpr millis DELAY_ON_ERROR_MS = 500;
std::shared_ptr<CyphalInterface> get_interface();
void in_loop_reporting(millis);
void setup_subscriptions();
void cyphal_loop();
void start_cyphal();
void set_cyphal_mode(uint8_t mode);

// common.cpp
micros micros_64();
millis millis_32();
void start_timers();

// app.cpp
std::shared_ptr<VBDrive> get_motor();
EEPROM& get_eeprom();

//fdcan.cpp
extern FDCAN_HandleTypeDef hfdcan1;

// config.cpp
struct VBDriveConfig: public BaseConfigData {
    static constexpr uint32_t TYPE_ID = 0x22AAABBB;
    // NAN means not set
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
    float filter_a = 0.0f;
    float filter_g1 = NAN;
    float filter_g2 = NAN;
    float filter_g3 = NAN;

    VBDriveConfig() {
        type_id = VBDriveConfig::TYPE_ID;
    }

    bool are_required_params_set();

    void print_self(UARTResponseAccumulator& responses);
    void get(const std::string& param, UARTResponseAccumulator& responses);
    bool set(const std::string& param, std::string& value, UARTResponseAccumulator& responses);
};
using AppConfigT = AppConfigurator<VBDriveConfig, sizeof(CalibrationData) + 1>;
AppConfigT& get_app_config();
void configure_fdcan(FDCAN_HandleTypeDef*);
inline float value_or_default(float value, float default_value) {
    return std::isnan(value) ? default_value : value;
}
