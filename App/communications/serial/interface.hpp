#pragma once

#include "config/config.hpp"
#include "state_manager/state_manager.h"
#include <voltbro/config/serial/serial.h>
#include <voltbro/motors/bldc/foc/foc.hpp>
#include <array>
#include <climits>
#include <cstring>
#include <cstdlib>
#include <tuple>

inline constexpr size_t UART_TX_BUFFER_SIZE = 512;
extern char serial_tx_buffer[UART_TX_BUFFER_SIZE];

inline void serial_wait_for_uart() {
    while (huart2.gState == HAL_UART_STATE_BUSY_TX || huart2.gState == HAL_UART_STATE_BUSY) {}
}

inline void serial_send_message(const char* format, ...) {
    serial_wait_for_uart();
    va_list args;
    va_start(args, format);
    const int written = npf_vsnprintf(serial_tx_buffer, sizeof(serial_tx_buffer), format, args);
    va_end(args);
    if (written > 0) HAL_UART_Transmit_DMA(&huart2, reinterpret_cast<uint8_t*>(serial_tx_buffer),
                                         std::min(static_cast<size_t>(written), sizeof(serial_tx_buffer)-1));
}

inline std::optional<std::tuple<std::string_view, std::string_view>> split_parameter(std::string_view command) {
    const auto colon = command.find(':');
    if (colon == std::string_view::npos) return std::nullopt;
    return std::make_tuple(command.substr(0, colon), command.substr(colon+1));
}

static constexpr size_t PARSED_VALUE_MAX_SIZE = 31;

inline bool parse_serial_number(std::string_view str, int& out_val) {
    if (str.empty()) return false;
    if (str.size() > PARSED_VALUE_MAX_SIZE) return false;

    char buffer[PARSED_VALUE_MAX_SIZE + 1];
    memcpy(buffer, str.data(), str.size());
    buffer[str.size()] = '\0';

    char* endptr = nullptr;
    long val = strtol(buffer, &endptr, 10);

    // Check for conversion errors
    if (endptr != buffer + str.size()) return false;
    if (val < INT32_MIN || val > INT32_MAX) return false;

    out_val = static_cast<int>(val);
    return true;
}

inline bool parse_serial_number(std::string_view str, float& out_val) {
    if (str.empty()) return false;
    if (str.size() > PARSED_VALUE_MAX_SIZE) return false;

    char buffer[PARSED_VALUE_MAX_SIZE + 1];
    memcpy(buffer, str.data(), str.size());
    buffer[str.size()] = '\0';

    char* endptr = nullptr;
    float val = strtof(buffer, &endptr);

    if (endptr != buffer + str.size()) return false;
    if (val == HUGE_VALF || val == -HUGE_VALF) return false;

    out_val = val;
    return true;
}


inline void serial_get_parameter(const DriveConfig& config, std::string_view param, UARTResponseAccumulator& responses) {
    const auto* definition = find_parameter(param);
    if (!definition) {
        responses.append("%.*s ERROR: Unknown parameter\r\n", static_cast<int>(param.size()), param.data());
        return;
    }
    ParameterValue value{};
    if (!read_parameter(config, definition->id, value)) {
        responses.append("%.*s ERROR: Parameter unavailable\r\n", static_cast<int>(param.size()), param.data());
        return;
    }
    switch (definition->type) {
        case ParameterType::REAL32:
            responses.append("%s:%f\n\r", definition->name.data(), std::get<float>(value));
            break;
        case ParameterType::NATURAL32:
            responses.append("%s:%lu\n\r", definition->name.data(), std::get<uint32_t>(value));
            break;
        case ParameterType::INTEGER32:
            responses.append("%s:%ld\n\r", definition->name.data(), std::get<int32_t>(value));
            break;
        case ParameterType::BIT:
            responses.append("%s:%u\n\r", definition->name.data(), std::get<bool>(value) ? 1U : 0U);
            break;
        case ParameterType::STRING:
            responses.append("%s:%.*s\n\r", definition->name.data(), static_cast<int>(std::get<std::string_view>(value).size()), std::get<std::string_view>(value).data());
            break;
    }
}

inline bool serial_set_parameter(DriveConfig& config, std::string_view param, std::string_view input, UARTResponseAccumulator& responses, bool apply_runtime) {
    const auto* definition = find_parameter(param);
    if (!definition) {
        responses.append("%.*s ERROR: Unknown parameter\r\n", static_cast<int>(param.size()), param.data());
        return false;
    }
    if (!definition->is_persistent) {
        responses.append("%.*s ERROR: %s\r\n", static_cast<int>(param.size()), param.data(),
                         definition->is_mutable ? "Parameter unavailable" : "Read-only parameter");
        return false;
    }

    ParameterValue value{};
    int integer = 0;
    if (definition->type == ParameterType::REAL32) {
        if (!parse_serial_number(input, value.emplace<float>())) {
            responses.append("%.*s ERROR: Invalid value\r\n", static_cast<int>(param.size()), param.data());
            return false;
        }
    }
    else if (definition->type == ParameterType::NATURAL32 || definition->type == ParameterType::INTEGER32) {
        if (!parse_serial_number(input, integer) || (definition->type == ParameterType::NATURAL32 && integer < 0)) {
            responses.append("%.*s ERROR: Invalid value\r\n", static_cast<int>(param.size()), param.data());
            return false;
        }
        if (definition->type == ParameterType::INTEGER32) value = static_cast<int32_t>(integer);
        else value = static_cast<uint32_t>(integer);
    }
    else if (definition->type == ParameterType::STRING) {
        value = input;
    }
    else {
        responses.append("%.*s ERROR: Invalid value\r\n", static_cast<int>(param.size()), param.data());
        return false;
    }

    if (write_persistent_parameter(config, definition->id, value, apply_runtime) != ParameterWriteResult::OK) {
        responses.append("%.*s ERROR: Invalid value\r\n", static_cast<int>(param.size()), param.data());
        return false;
    }
    if (definition->type == ParameterType::REAL32) {
        responses.append("%s:%f OK\r\n", definition->name.data(), std::get<float>(value));
    }
    else if (definition->type == ParameterType::INTEGER32) {
        responses.append("%s:%ld OK\r\n", definition->name.data(), std::get<int32_t>(value));
    }
    else if (definition->type == ParameterType::STRING) {
        responses.append("%s:%.*s OK\r\n", definition->name.data(),
                         static_cast<int>(input.size()), input.data());
    }
    else {
        responses.append("%s:%lu OK\r\n", definition->name.data(), std::get<uint32_t>(value));
    }
    return true;
}
inline void serial_print_config(const DriveConfig& config, UARTResponseAccumulator& responses) {
    for (const auto& definition : PARAMETER_CATALOG) {
        if (definition.is_persistent) {
            serial_get_parameter(config, definition.name, responses);
        }
    }
    responses.append("was configured:%s\n\r", config.base.was_configured ? "true" : "false");
    responses.append("are all required params set: %s\n\r", config.are_required_params_set() ? "true" : "false");
}

inline void serial_print_help(UARTResponseAccumulator& responses) {
    responses.append("Commands are case-sensitive; terminate with CR or LF.\r\n");
    responses.append("INFO - repeat startup information (current config)\r\nHELP - list commands\r\n");
    responses.append("CONFIG - stop motor, stage config\r\nEXIT - discard staged changes\r\n");
    responses.append("SAVE - save config and exit\r\nRESET - stage defaults (CONFIG)\r\nAPPLY - save pending config and reboot\r\n");
    responses.append("CALIBRATE - isolated calibration; input discarded until done\r\nSTOP - set voltage target to zero\r\n");
    responses.append("mit_cmd: <pos> <vel> <torq> <p_gain> <v_gain> (RUNNING)\r\n");
    responses.append("servo_cmd: <type> <value> [command_idx] (RUNNING); types 0..6\r\n");
    responses.append("log_on - state at 100 Hz (RUNNING)\r\nlog_off - disable state log\r\n");
    responses.append("<parameter>:? - read\r\n<parameter>:<value> - write config in CONFIG\r\n");
    responses.append("is_on:0/1 - disable/enable driver\r\nbootloader:1 - reboot to loader without saving; 0 - no action\r\n");
    responses.append("No commands are processed during calibration.\r\n");
}

inline bool serial_control_target(std::string_view param, std::string_view value) {
    std::array<std::string_view, 5> args{};
    size_t count = 0;
    while (!value.empty() && count < args.size()) {
        const auto first = value.find_first_not_of(" \t");
        if (first == std::string_view::npos) { value = {}; break; }
        value.remove_prefix(first);
        const auto end = value.find_first_of(" \t");
        args[count++] = value.substr(0, end);
        value.remove_prefix(end == std::string_view::npos ? value.size() : end);
    }
    bool valid = value.find_first_not_of(" \t") == std::string_view::npos;
    if (param == "mit_cmd") {
        std::array<float, 5> numbers{};
        valid &= count == 5;
        for (size_t i = 0; valid && i < numbers.size(); ++i) {
            valid = parse_serial_number(args[i], numbers[i]) && std::isfinite(numbers[i]);
        }
        if (valid) valid = apply_mit_command(FOCTarget{
            .torque = numbers[2], .angle = numbers[0], .velocity = numbers[1],
            .angle_kp = numbers[3], .velocity_kp = numbers[4]});
    } else {
        int type = -1;
        int index = -1;
        float target = 0;
        valid = valid && (count == 2 || count == 3) && parse_serial_number(args[0], type) &&
            type >= 0 && type <= 6 && parse_serial_number(args[1], target) && std::isfinite(target);
        if (valid && count == 3) valid = parse_serial_number(args[2], index) && index >= 0 && index <= 255;
        if (valid) valid = apply_servo_command(static_cast<uint8_t>(type), target,
                                               count == 3, static_cast<uint8_t>(index));
    }
    return valid;
}
