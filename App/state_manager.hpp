#pragma once

#include <voltbro/config/serial/serial.h>
#include <voltbro/motors/bldc/vbdrive/vbdrive.hpp>

VBDrive* get_motor();

namespace VBDriveDefaults {
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

struct __attribute__((packed)) VBDriveConfig: public BaseConfigData {
    static constexpr uint32_t TYPE_ID = 0x44AAABFE;
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
    float I_lpf_coefficient = NAN;
    AngleEncoderType angle_encoder = AngleEncoderType::ROTOR;

    VBDriveConfig(): BaseConfigData() {
        type_id = VBDriveConfig::TYPE_ID;
        angle_encoder = AngleEncoderType::ROTOR;
    }

    bool are_required_params_set() override;

    void print_self(UARTResponseAccumulator& responses);
    void get(const std::string& param, UARTResponseAccumulator& responses);
    bool set(const std::string& param, std::string& value, UARTResponseAccumulator& responses);
};

constexpr size_t CALIBRATION_PLACEMENT = 0;
constexpr size_t CONFIG_PLACEMENT = CALIBRATION_PLACEMENT + sizeof(CalibrationData) + 1;
constexpr size_t IND_SENSOR_STATE_PLACEMENT = CONFIG_PLACEMENT + sizeof(VBDriveConfig) + 1;

struct CommandState: AppState {
    static constexpr AppStateT NOT_CALIBRATED{4};
    static constexpr AppStateT CALIBRATING{5};
    static constexpr AppStateT TESTING{6};
};

class DriveStateController: public AppConfigurator<CommandState, VBDriveConfig, CONFIG_PLACEMENT> {
protected:
    static constexpr std::string_view TEST_COMMAND = "TEST";
    static constexpr std::string_view CALIBRATE_COMMAND = "CALIBRATE";
    static constexpr std::string_view STOP_COMMAND = "STOP";
    static constexpr std::string_view VEL_PARAM = "do_vel";
    static constexpr std::string_view ANGLE_PARAM = "do_ang";
    static constexpr std::string_view FREE_COMMAND = "do_free";
    static constexpr std::string_view START_LOGGING_COMMAND = "log_on";
    static constexpr std::string_view STOP_LOGGING_COMMAND = "log_off";

    const std::array<std::string_view, 3> editable_in_test_mode = {
        "min_ang",
        "max_ang",
        "ang_off"
    };

    bool _is_logging = false;

public:
    using BaseConfigurator = AppConfigurator<CommandState, VBDriveConfig, CONFIG_PLACEMENT>;  // for brevity
    using BaseConfigurator::AppConfigurator;  // inherit constructors
    using BaseConfigurator::process_command;  // inherit base method

    bool is_logging() {
        return BaseConfigurator::app_state == CommandState::TESTING && _is_logging;
    }

    void send_message_blocking(const char* fmt, ...) {
        size_t buffer_size = strlen(fmt) + 15;  // should be enough for param expansion
        char buffer[buffer_size];

        va_list args;
        va_start(args, fmt);
        int written = npf_vsnprintf(buffer, buffer_size, fmt, args);
        va_end(args);

        if (written == 0) {
            return;
        }

        wait_for_uart();
        HAL_UART_Transmit(huart, reinterpret_cast<uint8_t*>(buffer), written, 1000);
    }

    void set_calibration_finished() {
        BaseConfigurator::app_state = CommandState::RUNNING;
        char message[] = "Calibration finished\n\r\0";
        send_message_blocking(message);
    }

    bool is_calibration_allowed() const {
        return BaseConfigurator::app_state == CommandState::CALIBRATING;
    }

    bool is_app_running() const override {
        return BaseConfigurator::is_app_running() || BaseConfigurator::app_state == CommandState::TESTING;
    }

    void process_command(std::string& command, UARTResponseAccumulator& responses) override {
        if (BaseConfigurator::app_state == CommandState::RUNNING) {
            if (command == TEST_COMMAND) {
                BaseConfigurator::app_state = CommandState::TESTING;
                responses.append("Entering TEST mode\n\r");
                return;
            }
        }
        if (BaseConfigurator::app_state == CommandState::TESTING) {
            handle_testing_mode(command, responses);
            return;
        }
        BaseConfigurator::process_command(command, responses);
    }

    void handle_testing_mode(std::string& command, UARTResponseAccumulator& responses) {
        auto motor = get_motor();

        if (command == STOP_COMMAND) {
            responses.append("Stopping TEST mode\n\r");
            motor->set_foc_point(FOCTarget{0});
            app_state = CommandState::RUNNING;
            _is_logging = false;
        }
        else if (command == START_LOGGING_COMMAND) {
            _is_logging = true;
        }
        else if (command == STOP_LOGGING_COMMAND) {
            _is_logging = false;
        }
        else if (command == FREE_COMMAND) {
            motor->set_foc_point(FOCTarget{0});
            responses.append("No effort mode\r\n");
        }
        else {
            auto values = split_parameter(command);
            if (!values) {
                responses.append("ERROR: Unknown command\n\r");
                return;
            }
            auto& [param, value] = *values;
            float setpoint = 0;
            bool is_converted = safe_stof(value, setpoint);
            if (!is_converted) {
                responses.append("ERROR: Unknown command\n\r");
                return;
            }
            if (param == VEL_PARAM) {
                motor->set_foc_point(FOCTarget{
                    .torque = 0,
                    .angle = 0,
                    .velocity = setpoint,
                    .angle_kp = 0,
                    .velocity_kp = 0.5f
                });
                responses.append("Set velocity: <%f>\n\r", setpoint);
            }
            else if (param == ANGLE_PARAM) {
                motor->set_foc_point(FOCTarget{
                    .torque = 0,
                    .angle = setpoint,
                    .velocity = 0,
                    .angle_kp = 7.0f,
                    .velocity_kp = 0.5
                });
                responses.append("Set angle: <%f>\n\r", setpoint);
            }
            else {
                bool is_processed = process_parameter(command, responses);
                if (!is_processed) {
                    responses.append("ERROR: Unknown command\n\r");
                }
                else {
                    // Apply limits dynamically in this session
                    auto new_limits = motor->get_limits();
                    new_limits.user_angle_offset = config_data.angle_offset;
                    new_limits.user_position_lower_limit = config_data.min_angle;
                    new_limits.user_position_upper_limit = config_data.max_angle;
                    motor->set_limits(new_limits);
                }
            }
        }
    }
};
