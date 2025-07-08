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
    static constexpr uint32_t TYPE_ID = 0xAAAAEF01;
    int test_param1 = 0;
    float test_param2 = 0.0f;

    VBDriveConfig() {
        type_id = VBDriveConfig::TYPE_ID;
    }

    bool are_required_params_set();

    void get(std::string& param, UARTResponseAccumulator& responses);
    bool set(std::string& param, std::string& value, UARTResponseAccumulator& responses);
};
using AppConfigT = AppConfigurator<VBDriveConfig, sizeof(CalibrationData) + 1>;
AppConfigT& get_app_config();
void configure_fdcan(FDCAN_HandleTypeDef*);
