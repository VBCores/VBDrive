#pragma once

#include "main.h"
#include "stm32g4xx_hal.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

#include "state_manager.hpp"

// state_manager.hpp
// CALIBRATION_PLACEMENT
// CONFIG_PLACEMENT
// IND_SENSOR_STATE_PLACEMENT

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 50;

// NOTE: due to RAM constraints, this buffer is first used for calibration, then reused for Cyphal queue.
//       It should be big enough for both purposes. For calibration, it should be >=(2048+1)*4 for guaranteed alignment
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
// actions (in app.cpp)
bool is_able_to_calibrate();
bool do_calibrate();

//fdcan.cpp
extern FDCAN_HandleTypeDef hfdcan1;

// state_manager.cpp
DriveStateController& get_app_manager();
void configure_fdcan(FDCAN_HandleTypeDef*);
void reboot_to_bootloader();

template <typename T>
inline T value_or_default(T value, T default_value) {
    return std::isnan(value) ? default_value : value;
}

template <typename T>
inline T value_or_default(T value, T default_value, T not_set_value) {
    return (value == not_set_value) ? default_value : value;
}

//command_mode.cpp
void start_uart_recv_it();
