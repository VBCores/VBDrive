#pragma once

#include <voltbro/config/serial/serial.h>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

#include <cstddef>
#include "parameters.hpp"

VBDrive* get_motor();
bool apply_mit_command(FOCTarget target);
bool apply_servo_command(uint8_t type, float value);
void reboot_to_bootloader();
bool parse_serial_number(std::string_view input, int& value);
bool parse_serial_number(std::string_view input, float& value);

namespace VBDriveDefaults {
    inline constexpr float SERVO_POS_P_GAIN = 150.0f;
    inline constexpr float SERVO_POS_I_GAIN = 200.0f;
    inline constexpr float SERVO_POS_D_GAIN = 10.0f;
    inline constexpr float SERVO_VEL_P_GAIN = 30.0f;
    inline constexpr float SERVO_VEL_I_GAIN = 60.0f;
    inline constexpr float SERVO_TRANSIENT_VEL = 0.0f;
    inline constexpr uint32_t SERVO_TRANSIENT_FORM = 1; // LINE_TRAJ; not implemented yet.
    inline constexpr float MAX_VOLTAGE = 50.0f;
    inline constexpr uint8_t GEAR_RATIO = 36;
    inline constexpr float TORQUE_CONST = 1.0f;
    inline constexpr float ANGLE_OFFSET = 0.0f;
    inline constexpr float PID_KP = 4.0f;
    inline constexpr float PID_KI = 1600.0f;
    inline constexpr float PID_KD = 0.0f;
    inline constexpr float FILTER_A = 0.0f;
    inline constexpr float FILTER_G1 = 0.015700989410003974f;
    inline constexpr float FILTER_G2 = 3.925227776360174f;
    inline constexpr float FILTER_G3 = 387.54711795263574f;
    inline constexpr float I_LPF = 0.0925f;
}  // namespace VBDriveDefaults

inline constexpr uint32_t VBDRIVE_CONFIG_TYPE_ID = 0x44AAABFF;

struct __attribute__((packed)) VBDriveConfig: public BaseConfigData {
    static constexpr uint32_t TYPE_ID = VBDRIVE_CONFIG_TYPE_ID;
    uint8_t gear_ratio = 0;
    int32_t angle_direction = 1;
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
    float I_lpf_coefficient = NAN;
    AngleEncoderType angle_encoder = AngleEncoderType::ROTOR;
    float servo_pos_p_gain = NAN;
    float servo_pos_i_gain = NAN;
    float servo_pos_d_gain = NAN;
    float servo_vel_p_gain = NAN;
    float servo_vel_i_gain = NAN;
    uint32_t servo_transient_form = 0; // Unset; integer equivalent of NAN.
    float servo_transient_vel = NAN;

    VBDriveConfig(): BaseConfigData() {
        type_id = VBDriveConfig::TYPE_ID;
        angle_encoder = AngleEncoderType::ROTOR;
    }

    bool are_required_params_set();

    void print_self(UARTResponseAccumulator& responses);
    void get(std::string_view param, UARTResponseAccumulator& responses);
    bool set(std::string_view param, std::string_view value, UARTResponseAccumulator& responses, bool apply_runtime = false);
    void apply_servo_config() const;
};

static_assert(sizeof(BaseConfigData) == 8);
static_assert(offsetof(BaseConfigData, was_configured) == 0);
static_assert(offsetof(BaseConfigData, node_id) == 1);
static_assert(offsetof(BaseConfigData, fdcan_nominal_baud) == 2);
static_assert(offsetof(BaseConfigData, fdcan_data_baud) == 3);
static_assert(offsetof(BaseConfigData, type_id) == 4);

constexpr size_t CONFIG_PLACEMENT = 0;
constexpr size_t CALIBRATION_PLACEMENT = CONFIG_PLACEMENT + sizeof(VBDriveConfig) + 1;
constexpr size_t IND_SENSOR_STATE_PLACEMENT = CALIBRATION_PLACEMENT + sizeof(CalibrationData) + 1;

struct CommandState: AppState {
    static constexpr AppStateT NOT_CALIBRATED{4};
    static constexpr AppStateT CALIBRATING{5};
};

class DriveStateController: public AppConfigurator<CommandState, VBDriveConfig, CONFIG_PLACEMENT, std::string_view> {
protected:
    static constexpr std::string_view CALIBRATE_COMMAND = "CALIBRATE";
    static constexpr std::string_view STOP_COMMAND = "STOP";
    static constexpr std::string_view START_LOGGING_COMMAND = "log_on";
    static constexpr std::string_view STOP_LOGGING_COMMAND = "log_off";

    bool _is_logging = false;
    CommandState::ValueT state_before_config = CommandState::INIT;
    VBDriveConfig config_before_config;

public:
    using BaseConfigurator = AppConfigurator<CommandState, VBDriveConfig, CONFIG_PLACEMENT, std::string_view>;  // for brevity
    using BaseConfigurator::AppConfigurator;  // inherit constructors
    using BaseConfigurator::process_command;  // inherit base method

    // Cyphal must not read or persist the Serial CONFIG staging area.
    VBDriveConfig& get_committed_config() {
        return app_state == CommandState::CONFIG ? config_before_config : config_data;
    }

    void set_state(CommandState::ValueT state) {
        if (state != CommandState::RUNNING) _is_logging = false;
        BaseConfigurator::set_state(state);
    }

    bool is_logging() const {
        return app_state == CommandState::RUNNING && _is_logging;
    }

    void send_message(const char* fmt, ...) {
        wait_for_uart();
        auto& buffer = uart_tx_buffer;
        constexpr size_t buffer_size = sizeof(uart_tx_buffer);

        va_list args;
        va_start(args, fmt);
        int written = npf_vsnprintf(buffer, buffer_size, fmt, args);
        va_end(args);

        if (written <= 0) {
            return;
        }

        HAL_UART_Transmit_DMA(huart, reinterpret_cast<uint8_t*>(buffer), std::min(static_cast<size_t>(written), buffer_size - 1));
    }

    void set_calibration_finished() {
        BaseConfigurator::app_state = CommandState::RUNNING;
        char message[] = "Calibration finished\n\r\0";
        send_message(message);
    }

    bool is_calibration_allowed() const {
        return BaseConfigurator::app_state == CommandState::CALIBRATING;
    }

    void process_command(std::string_view command, UARTResponseAccumulator& responses) override {
        if (command == "INFO") {
            print_info(responses);
            return;
        }
        if (command == "HELP") {
            responses.append("Commands are case-sensitive; terminate with CR or LF.\r\n");
            responses.append("INFO - repeat startup information (current config)\r\nHELP - list commands\r\n");
            responses.append("CONFIG - stop motor, stage config\r\nEXIT - discard staged changes\r\n");
            responses.append("SAVE - save config and exit\r\nRESET - stage defaults (CONFIG)\r\nAPPLY - save pending config and reboot\r\n");
            responses.append("CALIBRATE - isolated calibration; input discarded until done\r\nSTOP - set voltage target to zero\r\n");
            responses.append("mit_cmd: <pos> <vel> <torq> <p_gain> <v_gain> (RUNNING)\r\n");
            responses.append("servo_cmd: <type> <value> (RUNNING); 0 velocity, 1 torque, 2 position, 3 voltage\r\n");
            responses.append("log_on - state at 100 Hz (RUNNING)\r\nlog_off - disable state log\r\n");
            responses.append("<parameter>:? - read\r\n<parameter>:<value> - write config in CONFIG\r\n");
            responses.append("is_on:0/1 - disable/enable driver\r\nbootloader:1 - reboot to loader without saving; 0 - no action\r\n");
            responses.append("No commands are processed during calibration.\r\n");
            return;
        }
        if (command == STOP_COMMAND) {
            if (auto motor = get_motor()) motor->set_voltage_point(0.0f);
            responses.append("OK: STOP\r\n");
            return;
        }
        if (command == STOP_LOGGING_COMMAND) {
            _is_logging = false;
            responses.append("OK: log_off\r\n");
            return;
        }
        if (command == START_LOGGING_COMMAND) {
            if (app_state != CommandState::RUNNING) {
                responses.append("ERROR: RUNNING mode required\r\n");
                return;
            }
            _is_logging = true;
            responses.append("OK: log_on\r\n");
            return;
        }
        auto values = BaseConfigurator::split_parameter(command);
        if (values) {
            auto [param, value] = *values;
            if (param == "mit_cmd" || param == "servo_cmd") {
                if (app_state != CommandState::RUNNING) {
                    responses.append("ERROR: RUNNING mode required\r\n");
                    return;
                }
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
                    float target = 0;
                    valid = valid && count == 2 && parse_serial_number(args[0], type) &&
                        type >= 0 && type <= 3 && parse_serial_number(args[1], target) && std::isfinite(target);
                    if (valid) valid = apply_servo_command(static_cast<uint8_t>(type), target);
                }
                if (!valid) {
                    record_invalid_command();
                    responses.append("ERROR: Invalid target\r\n");
                } else responses.append("OK: %s\r\n", param == "mit_cmd" ? "mit_cmd" : "servo_cmd");
                return;
            }
            if (value == "?") {
                config_data.get(param, responses);
                return;
            }
            const auto* definition = find_parameter(param);
            if (!definition) {
                responses.append("ERROR: Unknown parameter\r\n");
                return;
            }
            if (definition && !definition->is_mutable) {
                responses.append("ERROR: Read-only parameter\n\r");
                return;
            }
            if (definition && !definition->is_persistent && definition->is_mutable) {
                if (definition->id == ParameterId::IS_ON && BaseConfigurator::app_state != CommandState::RUNNING) {
                    responses.append("ERROR: RUNNING mode required\n\r");
                    return;
                }
                bool enabled = false;
                if (value == "0") {
                    enabled = false;
                }
                else if (value == "1") {
                    enabled = true;
                }
                else {
                    responses.append("ERROR: Invalid value\n\r");
                    return;
                }
                ParameterValue parameter_value = enabled;
                if (write_runtime_parameter(definition->id, parameter_value) == ParameterWriteResult::OK) {
                    responses.append("OK: %s:%u\n\r", definition->name.data(), enabled ? 1U : 0U);
                }
                else {
                    responses.append("ERROR: Parameter unavailable\n\r");
                }
                return;
            }
            if (definition && definition->is_persistent && BaseConfigurator::app_state != CommandState::CONFIG) {
                responses.append("ERROR: CONFIG mode required\n\r");
                return;
            }
        }
        if (!values && command != CONFIG_COMMAND && command != "EXIT" &&
            command != SAVE_COMMAND && command != "APPLY" && command != "RESET") {
            responses.append("ERROR: Unknown command\r\n");
            return;
        }
        if (command == CONFIG_COMMAND) {
            if (app_state == CommandState::CONFIG) {
                responses.append("CONFIG MODE ENABLED\n\r");
                return;
            }
            _is_logging = false;
            state_before_config = app_state;
            config_before_config = config_data;
        }
        if (app_state == CommandState::CONFIG && command == "EXIT") {
            config_data = config_before_config;
            do_save = false;
            app_state = state_before_config;
            if (app_state == CommandState::RUNNING) turn_on();
            responses.append("CONFIG MODE EXITED, CHANGES DISCARDED\n\r");
            return;
        }
        const bool apply_servo = app_state == CommandState::CONFIG && command == SAVE_COMMAND;
        const bool pending_save = do_save;
        BaseConfigurator::process_command(command, responses);
        if (apply_servo) config_data.apply_servo_config();
        if (values && app_state == CommandState::CONFIG) do_save |= pending_save;
    }

};
