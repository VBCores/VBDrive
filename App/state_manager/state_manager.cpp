#include "state_manager.h"
#include "app.h"
#include "profiling.hpp"
#include "communications/serial/interface.hpp"
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>
#include <voltbro/eeprom/eeprom.hpp>

DriveStateController::DriveStateController(EEPROM& eeprom) : configs(eeprom, BASE_CONFIG_DEFAULTS, CONFIG_PLACEMENT) {}

static DriveStateController drive_state_controller(get_eeprom());
DriveStateController& get_app_manager() { return drive_state_controller; }
DriveConfig& DriveStateController::get_config() { return configs.get_config(); }
DriveConfig& DriveStateController::get_committed_config() { return configs.get_committed_config(); }
uint8_t DriveStateController::get_node_id() const { return configs.get_committed_config().base.node_id; }
CommandState DriveStateController::get_state() const { return app_state; }
bool DriveStateController::is_app_running() const { return app_state == CommandState::RUNNING; }
bool DriveStateController::is_logging() const { return is_app_running() && logging; }

ParameterWriteResult DriveStateController::set_motor_enabled(bool enabled) {
    auto motor = get_motor();
    if (!motor) return ParameterWriteResult::UNAVAILABLE;
    if (enabled && !is_app_running()) return ParameterWriteResult::INVALID;
    if (!enabled) {
        if (configs.is_editing()) restore_motor_enable = false;
        motor->reset_servo_input();
    }
    return motor->set_state(enabled) == HAL_OK ? ParameterWriteResult::OK : ParameterWriteResult::INVALID;
}

void DriveStateController::set_state(CommandState state) {
    if (state != CommandState::RUNNING) logging = false;
    app_state = state;
}

void DriveStateController::mark_config_changed() { configs.mark_committed_changed(); }

void DriveStateController::persist_pending_config() {
    if (!configs.save_pending()) Error_Handler();
}

void DriveStateController::init() {
    if (!configs.load()) Error_Handler();
    // Load communication settings before the first application response or RX DMA.
    if (huart2.Init.BaudRate != get_config().base.serial_baud) {
        serial_wait_for_uart();
        huart2.Init.BaudRate = get_config().base.serial_baud;
        if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    }
    if (!configs.save()) Error_Handler();
    if (get_config().base.was_configured && get_config().are_required_params_set()) {
        set_state(CommandState::RUNNING);
    }
    process_command("INFO");
}

void DriveStateController::process_command(std::string_view command) {
    const auto end = command.find_last_not_of(" \t\n\r");
    if (end == std::string_view::npos) return;
    command = command.substr(0, end+1);
    UARTResponseAccumulator response(&huart2, serial_tx_buffer, UART_TX_BUFFER_SIZE,
                                     command == "INFO" || command == "HELP");
    process_command(command, response);
}

void DriveStateController::process_command(std::string_view command, UARTResponseAccumulator& responses) {
    if (command == "CALIBRATE") {
        if (!is_able_to_calibrate()) responses.append("CALIBRATE ERROR: conditions not met\r\n");
        else {
            serial_send_message("CALIBRATE OK\r\n");
            const bool success = do_calibrate();
            serial_wait_for_uart();
            responses.append(success ? "CALIBRATE FINISH\r\n" : "CALIBRATE ERROR: failed\r\n");
        }
        return;
    }
    if (command == "INFO") {
        const auto& config = configs.get_config();
        responses.append("Got config_data type_id: <0x%08lX>\r\n\r\n", config.app.type_id);
        serial_print_config(config, responses);
        responses.append(config.base.was_configured && config.are_required_params_set()
            ? "Controller is configured, starting\r\n" : "Controller was not configured!\r\n");
        responses.append("See HELP for available commands\r\n");
        return;
    }
    if (command == "HELP") { serial_print_help(responses); return; }
    if (command == "STOP") {
        if (auto motor = get_motor()) {
            motor->reset_servo_input();
            motor->set_voltage_point(0.0f);
        }
        responses.append("STOP OK\r\n");
        return;
    }
    if (command == "log_off") {
        logging = false;
        responses.append("log_off OK\r\n");
        return;
    }
    if (command == "log_on") {
        if (!is_app_running()) responses.append("log_on ERROR: RUNNING mode required\r\n");
        else { logging = true; responses.append("log_on OK\r\n"); }
        return;
    }
    if (auto values = split_parameter(command)) {
        const auto [param, value] = *values;
        if (param == "mit_cmd" || param == "servo_cmd") {
            if (!is_app_running()) {
                responses.append("%.*s ERROR: RUNNING mode required\r\n", int(param.size()), param.data());
            } else if (!serial_control_target(param, value)) {
                record_invalid_command();
                responses.append("%.*s ERROR: Invalid target\r\n", int(param.size()), param.data());
            } else responses.append("%.*s OK\r\n", int(param.size()), param.data());
            return;
        }
        if (value == "?") { serial_get_parameter(configs.get_config(), param, responses); return; }
        const auto* definition = find_parameter(param);
        const char* error = nullptr;
        if (!definition) error = "Unknown parameter";
        else if (!definition->is_mutable) error = "Read-only parameter";
        else if (definition->is_persistent && !configs.is_editing()) error = "CONFIG mode required";
        else if (definition->id == ParameterId::IS_ON && value == "1" && !is_app_running()) error = "RUNNING mode required";
        if (error) {
            responses.append("%.*s ERROR: %s\r\n", int(param.size()), param.data(), error);
            return;
        }
        if (definition->is_persistent) {
            if (serial_set_parameter(configs.get_config(), param, value, responses, false)) configs.mark_changed();
        } else if (value != "0" && value != "1") {
            responses.append("%.*s ERROR: Invalid value\r\n", int(param.size()), param.data());
        } else if (write_runtime_parameter(definition->id, ParameterValue{value == "1"}) != ParameterWriteResult::OK) {
            responses.append("%.*s ERROR: Parameter unavailable\r\n", int(param.size()), param.data());
        } else responses.append("%s:%u OK\r\n", definition->name.data(), value == "1" ? 1U : 0U);
        return;
    }
    if (command == "CONFIG") {
        if (!configs.is_editing()) {
            state_before_config = app_state;
            configs.begin();
            set_state(CommandState::CONFIG);
            restore_motor_enable = false;
            if (auto motor = get_motor()) {
                restore_motor_enable = motor->is_on();
                motor->stop();
            }
        }
        responses.append("CONFIG OK: mode enabled\r\n");
        return;
    }
    if (command == "EXIT" && configs.is_editing()) {
        configs.discard();
        set_state(state_before_config);
        if (is_app_running() && restore_motor_enable) {
            if (set_motor_enabled(true) != ParameterWriteResult::OK) {
                responses.append("EXIT ERROR: driver enable failed; changes discarded\r\n");
                return;
            }
        }
        responses.append("EXIT OK: changes discarded\r\n");
        return;
    }
    if (command == "APPLY") {
        if (!configs.save()) Error_Handler();
        serial_wait_for_uart();
        char message[] = "APPLY OK\r\n";
        if (HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(message), sizeof(message)-1, 100) != HAL_OK) Error_Handler();
        NVIC_SystemReset();
        return;
    }
    if (command == "RESET" || command == "SAVE" || command == "EXIT") {
        if (!configs.is_editing()) return;
        if (command == "RESET") {
            configs.reset();
            responses.append("RESET OK: defaults staged; run APPLY to activate\r\n");
        } else {
            if (!configs.commit()) Error_Handler();
            set_state(CommandState::RUNNING);
            configs.get_config().app.apply_servo_config();
            if (restore_motor_enable) {
                if (set_motor_enabled(true) != ParameterWriteResult::OK) {
                    responses.append("SAVE ERROR: driver enable failed; config saved\r\n");
                    return;
                }
            }
            responses.append("SAVE OK: config saved; run APPLY to activate\r\n");
        }
        return;
    }
    responses.append("%.*s ERROR: Unknown command\r\n", int(command.size()), command.data());
}

bool apply_mit_command(FOCTarget target) {
    if (!get_app_manager().is_app_running()) return false;
    auto motor = get_motor();
    return motor && motor->set_foc_point(std::move(target));
}

bool apply_servo_command(uint8_t type, float value, bool indexed, uint8_t index) {
    if (!get_app_manager().is_app_running()) return false;
    auto motor = get_motor();
    if (!motor) return false;
    const bool accepted = motor->set_servo_command(type, value, indexed, index);
    VBDRIVE_PROFILE_RESULT(servo, accepted)
    return accepted;
}


void reboot_to_bootloader_if_requested() {
    if (!bootloader_reboot_pending) return;
    bootloader_reboot_pending = false;
    reboot_to_bootloader();
}
