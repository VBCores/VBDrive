#pragma once

#include "config/config.hpp"
#include <cstdint>
#include <string_view>

class UARTResponseAccumulator;
struct FOCTarget;

enum class CommandState : uint32_t { INIT, RUNNING, CONFIG, NOT_CALIBRATED, CALIBRATING };

class DriveStateController {
    DriveConfigManager configs;
    CommandState app_state = CommandState::INIT;
    CommandState state_before_config = CommandState::INIT;
    bool logging = false;

public:
    explicit DriveStateController(EEPROM& eeprom);
    void init();
    DriveConfig& get_config();
    DriveConfig& get_committed_config();
    uint8_t get_node_id() const;
    CommandState get_state() const;
    void set_state(CommandState state);
    bool is_app_running() const;
    bool is_logging() const;
    void mark_config_changed();
    void persist_pending_config();
    [[gnu::cold]] void process_command(std::string_view command);
    [[gnu::cold]] void process_command(std::string_view command, UARTResponseAccumulator& responses);
};

DriveStateController& get_app_manager();
bool apply_mit_command(FOCTarget target);
bool apply_servo_command(uint8_t type, float value, bool indexed = false, uint8_t index = 0);
void reboot_to_bootloader();
void reboot_to_bootloader_if_requested();
